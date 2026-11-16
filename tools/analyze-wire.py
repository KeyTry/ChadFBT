"""Summarize a local capture without printing network addresses or tracker IDs."""
import collections
import importlib.util
import json
import pathlib
import sys

spec = importlib.util.spec_from_file_location("tracker_probe", pathlib.Path(__file__).with_name("probe-trackers.py"))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
counts = collections.Counter()
trackers = {}
invalid = 0
datagrams = 0
for line in pathlib.Path(sys.argv[1]).open(encoding="utf-8"):
    try:
        event = json.loads(line)
        _, _, packets = probe.decode(bytes.fromhex(event["bytes_hex"]))
    except (ValueError, KeyError):
        invalid += 1
        continue
    datagrams += 1
    for kind, values in packets:
        counts[kind] += 1
        if kind == 3 and values.get("mac"):
            trackers[values["mac"]] = {key: values.get(key) for key in ("firmware", "imu_type", "protocol_version")}
print(json.dumps({"datagrams":datagrams, "decode_errors_or_partial_lines":invalid,
                  "handshaken_devices":len(trackers),
                  "firmware":sorted({v["firmware"] for v in trackers.values() if v["firmware"]}),
                  "imu_types":sorted({v["imu_type"] for v in trackers.values() if v["imu_type"] is not None}),
                  "protocol_versions":sorted({v["protocol_version"] for v in trackers.values() if v["protocol_version"] is not None}),
                  "payload_counts":dict(sorted(counts.items()))},indent=2))
