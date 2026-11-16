#!/usr/bin/env python3
"""Bounded LAN-only tracker receiver; never changes tracker configuration/firmware.

Run while other tracker servers are closed. Defaults to all active physical IPv4
interfaces on Windows and takes an exclusive UDP/6969 bind. Captures are local.
"""
import argparse
import collections
import datetime
import ipaddress
import json
import math
import os
import pathlib
import socket
import struct
import subprocess
import time


def configured_interfaces():
    if os.name != "nt":
        raise RuntimeError("Use --interface with an active local IPv4 CIDR on this OS")
    script = ("Get-NetIPAddress -AddressFamily IPv4 | Where-Object { "
              "$_.IPAddress -notmatch '^(127\\.|169\\.254\\.)' -and "
              "$_.AddressState -eq 'Preferred' } | "
              "Select-Object IPAddress,PrefixLength | ConvertTo-Json -Compress")
    result = subprocess.run(["powershell", "-NoProfile", "-Command", script],
                            check=True, text=True, capture_output=True)
    entries = json.loads(result.stdout or "[]")
    if isinstance(entries, dict):
        entries = [entries]
    return [ipaddress.ip_interface(f"{x['IPAddress']}/{x['PrefixLength']}")
            for x in entries]


def decode_payload(kind, payload):
    """Minimal inspection decoder. Production decoder lives in tracker-protocol."""
    if kind == 3:
        result = {}
        if not payload:
            return result
        if len(payload) < 29:
            raise ValueError("short handshake")
        values = struct.unpack_from(">7I", payload)
        result.update(board_type=values[0], imu_type=values[1], mcu_type=values[2],
                      protocol_version=values[6])
        length = payload[28]
        if len(payload) < 29 + length + 6:
            raise ValueError("short handshake firmware/MAC")
        result["firmware"] = payload[29:29 + length].rstrip(b"\0").decode("utf-8", "replace")
        result["mac"] = payload[29 + length:35 + length].hex(":")
        return result
    if kind == 17:
        sid, dtype, x, y, z, w, accuracy = struct.unpack_from(">BBffffB", payload)
        if not all(math.isfinite(v) for v in (x, y, z, w)):
            raise ValueError("nonfinite rotation")
        return dict(sensor_id=sid, data_type=dtype, quaternion_xyzw=[x, y, z, w], accuracy=accuracy)
    if kind == 4:
        x, y, z = struct.unpack_from(">fff", payload)
        return dict(sensor_id=payload[12] if len(payload) > 12 else 0, acceleration=[x, y, z])
    if kind == 15:
        if len(payload) < 2:
            raise ValueError("short sensor info")
        result = dict(sensor_id=payload[0], status=payload[1])
        if len(payload) >= 3:
            result["imu_type"] = payload[2]
        if len(payload) >= 6:
            result["rest_calibrated"] = bool(payload[5])
        return result
    if kind == 12:
        if len(payload) >= 8:
            voltage, level = struct.unpack_from(">ff", payload)
            return dict(voltage=voltage, level=level)
        return dict(level=struct.unpack_from(">f", payload)[0])
    if kind == 19:
        sid, rssi = struct.unpack_from(">Bb", payload)
        return dict(sensor_id=sid, rssi_dbm=rssi)
    if kind == 20:
        sid, temperature = struct.unpack_from(">Bf", payload)
        return dict(sensor_id=sid, temperature_c=temperature)
    if kind == 23:
        sid, *values = struct.unpack_from(">B7h", payload)
        return dict(sensor_id=sid, quaternion_xyzw=[v/32768 for v in values[:4]],
                    acceleration=[v/128 for v in values[4:]])
    if kind == 13:
        sid, bits = struct.unpack_from(">BB", payload)
        return dict(sensor_id=sid, tap_bits=bits)
    return {}


def decode(data):
    if len(data) < 12:
        raise ValueError("short header")
    kind, sequence = struct.unpack_from(">IQ", data)
    payload = data[12:]
    packets = []
    if kind in (100, 101):
        while payload:
            size_width, type_width = (2, 4) if kind == 100 else (1, 1)
            if len(payload) < size_width:
                raise ValueError("short bundle length")
            length = int.from_bytes(payload[:size_width], "big")
            payload = payload[size_width:]
            if length == 0:
                continue
            if length < type_width or len(payload) < length:
                raise ValueError("invalid bundle length")
            inner, payload = payload[:length], payload[length:]
            packet_type = int.from_bytes(inner[:type_width], "big")
            packets.append((packet_type, decode_payload(packet_type, inner[type_width:])))
    else:
        packets.append((kind, decode_payload(kind, payload)))
    return kind, sequence, packets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=25)
    parser.add_argument("--interface", action="append", default=[], help="active local IPv4 CIDR")
    parser.add_argument("--peer", action="append", default=[], help="known tracker IPv4 from local neighbor table; no scanning")
    parser.add_argument("--output", default="local-data/tracker-probe")
    parser.add_argument("--samples-per-type", type=int, default=4)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300 or not 0 <= args.samples_per_type <= 20:
        parser.error("duration must be 1..300 seconds; sample bound must be 0..20")
    configured = configured_interfaces()
    interfaces = [ipaddress.ip_interface(x) for x in args.interface] if args.interface else configured
    if not interfaces:
        parser.error("no configured IPv4 interface")
    if any(x not in configured or not x.ip.is_private for x in interfaces):
        parser.error("interfaces must be configured private IPv4 interfaces")
    destinations = sorted({str(x.network.broadcast_address) for x in interfaces})
    networks = [x.network for x in interfaces]
    if any(not any(ipaddress.ip_address(x) in n for n in networks) for x in args.peer):
        parser.error("known peers must be on a configured local interface subnet")
    local_ips = {str(x.ip) for x in configured}
    output = pathlib.Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Windows defaults can permit two binds; explicitly refuse port sharing.
    if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.1)
    try:
        sock.bind(("0.0.0.0", 6969))
    except OSError as exc:
        raise SystemExit(f"UDP 6969 unavailable; leave the existing owner running: {exc}")
    peers = {}
    start = time.monotonic()
    next_heartbeat = start
    next_discovery = start
    malformed = 0
    sampled = collections.Counter()
    header = lambda kind: struct.pack(">IQ", kind, 0)
    handshake = b"\x03Hey OVR =D 5".ljust(64, b"\0")
    with (output / "samples.jsonl").open("w", encoding="utf-8") as capture:
        while time.monotonic() - start < args.seconds:
            now = time.monotonic()
            if now >= next_discovery:
                for destination in destinations:
                    sock.sendto(header(0), (destination, 6969))
                # SlimeVR firmware accepts the special server handshake while
                # searching. Target only explicitly supplied known tracker IPs.
                for known_peer in args.peer:
                    if (known_peer, 6969) not in peers:
                        sock.sendto(handshake, (known_peer, 6969))
                next_discovery = now + 2
            if now >= next_heartbeat:
                for address in peers:
                    sock.sendto(header(1), address)
                next_heartbeat = now + 0.5
            try:
                data, address = sock.recvfrom(4096)
            except socket.timeout:
                continue
            if address[0] in local_ips or not any(ipaddress.ip_address(address[0]) in n for n in networks):
                continue
            try:
                kind, sequence, packets = decode(data)
            except (ValueError, struct.error, IndexError):
                malformed += 1
                continue
            if address not in peers and not any(p[0] == 3 for p in packets):
                continue
            peer = peers.setdefault(address, dict(alias=f"tracker-{len(peers)+1}",
                endpoint=list(address), packet_counts=collections.Counter(),
                first_seconds=now-start, last_seconds=now-start, latest={}, sequence_gaps=0,
                sequence_reorders=0, previous_sequence=None))
            previous = peer["previous_sequence"]
            # A reconnect handshake resets the session counter. Ping echoes have sequence 0.
            if kind == 3:
                peer["previous_sequence"] = sequence
            elif sequence > 0:
                if previous is not None and sequence > previous:
                    peer["sequence_gaps"] += max(0, sequence-previous-1)
                elif previous is not None and sequence <= previous:
                    peer["sequence_reorders"] += 1
                peer["previous_sequence"] = max(sequence, previous or 0)
            peer["last_seconds"] = now-start
            for packet_type, fields in packets:
                peer["packet_counts"][str(packet_type)] += 1
                peer["latest"][str(packet_type)] = fields
                if packet_type == 3:
                    sock.sendto(handshake, address)
                elif packet_type == 15:
                    sock.sendto(struct.pack(">IBB", 15, fields["sensor_id"], fields["status"]), address)
                elif packet_type == 22:
                    sock.sendto(header(22) + b"\x01", address)  # bundle100 support
            sample_key = (address, kind)
            if sampled[sample_key] < args.samples_per_type:
                capture.write(json.dumps(dict(elapsed_seconds=now-start, alias=peer["alias"],
                    endpoint=list(address), packet_type=kind, sequence=sequence,
                    datagram_hex=data.hex(), packets=packets)) + "\n")
                sampled[sample_key] += 1
    sock.close()
    summary = dict(captured_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        duration_seconds=time.monotonic()-start, malformed_datagrams=malformed, trackers=list(peers.values()))
    for peer in summary["trackers"]:
        elapsed = max(0.001, peer["last_seconds"] - peer["first_seconds"])
        peer["rates_hz"] = {k: round(v/elapsed, 2) for k,v in peer["packet_counts"].items()}
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(dict(trackers=len(peers), malformed_datagrams=malformed,
        summaries=[dict(alias=p["alias"], firmware=p["latest"].get("3",{}).get("firmware"),
          protocol=p["latest"].get("3",{}).get("protocol_version"),
          imu=p["latest"].get("3",{}).get("imu_type"), rates_hz=p["rates_hz"],
          sequence_gaps=p["sequence_gaps"], sequence_reorders=p["sequence_reorders"])
          for p in peers.values()]), indent=2))


if __name__ == "__main__":
    main()
