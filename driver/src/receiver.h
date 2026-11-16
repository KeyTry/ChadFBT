#pragma once

#include "openvr_driver.h"
#include "wire.h"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>

enum class TargetReadStatus { packet, empty, discard, error };
struct TargetRead {
    TargetReadStatus status = TargetReadStatus::empty;
    TargetFrame frame{};   // large enough for v2; a v1 datagram occupies the first 328 bytes
    size_t size = 0;
    bool loopback = false;
};

// Owned by exactly one thread (the driver's receive thread, or a test). No
// cross-process clock is assumed: all timestamps come from the receiver's own
// monotonic clock.
class TargetReceiver {
public:
    using Clock = std::chrono::steady_clock;
    using Time = Clock::time_point;
    // An accepted frame is live for this long after receipt (the app sends at 100 Hz).
    static constexpr auto timeout = std::chrono::milliseconds(100);
    // Trackers report disconnected only when no frame was accepted for longer than this.
    static constexpr auto disconnect_after = std::chrono::milliseconds(1000);

    explicit TargetReceiver(Time now) : last_poll_(now) {}
    void begin_poll(Time now) {
        if (now < last_poll_ || now - last_poll_ >= timeout) {
            flushing_ = true;
            have_ = false;
        }
        last_poll_ = now;
    }
    // The socket must be drained completely before new frames can become live.
    void drained() { flushing_ = false; }
    void overflow_or_error() { have_ = false; flushing_ = true; }
    bool flushing() const { return flushing_; }

    bool accept(const void* bytes, size_t count, bool loopback, Time now) {
        TargetFrame frame{};
        if (flushing_ || !decode(bytes, count, loopback, frame)) return false;
        if (have_watermark_ && now < received_) { overflow_or_error(); return false; }
        if (have_watermark_ && frame.header.session == last_.header.session &&
            frame.header.sequence <= last_.header.sequence) return false;
        if (have_watermark_ && frame.header.session != last_.header.session) {
            if (now - received_ < timeout) return false;
            for (size_t i = 0; i < retired_count_; ++i)
                if (retired_[i] == frame.header.session) return false;
            retired_[retired_next_] = last_.header.session;
            retired_next_ = (retired_next_ + 1) % retired_.size();
            if (retired_count_ < retired_.size()) ++retired_count_;
        }
        last_ = frame;
        received_ = now;
        have_ = true;
        have_watermark_ = true;
        return true;
    }

    // Remember only validated same-session sequence numbers discarded while
    // draining. Do not update the accepted frame or its lease timestamp.
    void observe_discarded(const void* bytes, size_t count, bool loopback) {
        TargetFrame frame{};
        if (!flushing_ || !have_watermark_ || !decode(bytes,count,loopback,frame)) return;
        if (frame.header.session == last_.header.session && frame.header.sequence > last_.header.sequence)
            last_.header.sequence = frame.header.sequence;
    }

    // Shared by the live driver and an offline synthetic socket. On a full
    // budget we cannot prove the queue is empty, so fail closed and drain it.
    // Returns true when at least one new frame was accepted.
    template<class Read>
    bool poll(Time now, Read read) {
        begin_poll(now);
        bool fresh = false;
        for (int n = 0; n < 64; ++n) {
            TargetRead item = read();
            switch (item.status) {
                case TargetReadStatus::packet:
                    if (flushing_) observe_discarded(&item.frame, item.size, item.loopback);
                    else fresh = accept(&item.frame, item.size, item.loopback, now) || fresh;
                    break;
                case TargetReadStatus::empty:
                    drained(); return fresh && have_;
                case TargetReadStatus::discard:
                    break;
                case TargetReadStatus::error:
                    overflow_or_error(); return false;
            }
        }
        overflow_or_error();
        return false;
    }

    // Disconnected: no frame accepted for > disconnect_after (or never).
    // Out of range: the app session is alive but this slot is invalid, the
    // lease expired, or the receiver is draining. Valid: a live, valid slot.
    vr::DriverPose_t pose(size_t slot, Time now) const {
        vr::DriverPose_t out{};
        out.qWorldFromDriverRotation.w = 1;
        out.qDriverFromHeadRotation.w = 1;
        out.qRotation.w = 1;
        out.result = vr::TrackingResult_Uninitialized;
        if (!have_watermark_ || now < received_ || now - received_ > disconnect_after) return out;
        out.deviceIsConnected = true;
        out.result = vr::TrackingResult_Running_OutOfRange;
        if (!have_ || flushing_ || slot >= 8 || now - received_ >= timeout) return out;
        const auto& src = last_.poses[slot];
        if (!src.valid) return out;
        const float* matrix = last_.header.raw_from_standing;
        out.poseIsValid = true;
        out.result = vr::TrackingResult_Running_OK;
        // The pose sample was pose_age_secs old when sent, plus time since receipt here.
        out.poseTimeOffset = -(double(last_.pose_age_secs) +
                               std::chrono::duration<double>(now - received_).count());
        for (int i = 0; i < 3; ++i)
            out.vecPosition[i] = matrix[i*4]*src.p[0] + matrix[i*4+1]*src.p[1] +
                                 matrix[i*4+2]*src.p[2] + matrix[i*4+3];
        rotate_vector(matrix, src.v, out.vecVelocity);
        // Angular velocity frame is unverified in SteamVR (see PROTOCOL.md), so it is opt-in.
        if (last_.header.flags & kFlagApplyAngularVelocity)
            rotate_vector(matrix, src.w, out.vecAngularVelocity);
        double a[4];
        quat_from_matrix(matrix, a);
        const float* b = src.q;
        double q[4] = {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
                       a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
                       a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
                       a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
        const double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        out.qRotation = {q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm};
        return out;
    }
private:
    // Accepts v2 (528 bytes) and, for older app builds, v1 (328 bytes) as zero velocity.
    static bool decode(const void* bytes, size_t count, bool loopback, TargetFrame& frame) {
        if (!loopback || !bytes) return false;
        if (count == sizeof(TargetFrame)) {
            std::memcpy(&frame, bytes, sizeof(frame));
            if (frame.header.version != 2 || (frame.header.flags & ~kFlagApplyAngularVelocity) ||
                !valid_pose_age(frame.pose_age_secs)) return false;
        } else if (count == sizeof(TargetFrameV1)) {
            TargetFrameV1 legacy{};
            std::memcpy(&legacy, bytes, sizeof(legacy));
            if (legacy.header.version != 1) return false;
            for (const auto& p : legacy.poses) if (!valid_pose(p)) return false;
            upgrade_v1(legacy, frame);
        } else {
            return false;
        }
        if (std::memcmp(frame.header.magic, "CFBP", 4) || frame.header.session == 0 ||
            !rigid(frame.header.raw_from_standing)) return false;
        for (const auto& p : frame.poses) if (!valid_pose(p)) return false;
        return true;
    }
    TargetFrame last_{};
    bool have_ = false;
    // Invalidation clears pose validity but not the replay watermark.
    bool have_watermark_ = false;
    std::array<uint64_t, 8> retired_{};
    size_t retired_count_ = 0;
    size_t retired_next_ = 0;
    bool flushing_ = false;
    Time received_{};
    Time last_poll_{};
};

// Decides which slot poses to submit, and when a slot first needs a SteamVR
// device. Owned by the receive thread (or a test).
//
// - A slot whose pose has never been valid requests nothing and submits nothing,
//   so an unused slot never appears in SteamVR.
// - The first valid pose of a slot calls request(slot) exactly once; the device
//   is added later on SteamVR's main thread (see SlotRegistry).
// - Once the slot's device is active: a valid pose is submitted once per newly
//   accepted frame (never re-stamped); an invalid state is submitted once per
//   transition and carries the slot's last valid position and rotation with zero
//   velocity, so nothing jumps to the tracking-space origin. A device index change
//   (late activation, reactivation) forces a submission of the current state.
class PosePublisher {
public:
    enum class State : unsigned char { unknown, disconnected, out_of_range, valid };
    static State classify(const vr::DriverPose_t& pose) {
        if (pose.poseIsValid) return State::valid;
        return pose.deviceIsConnected ? State::out_of_range : State::disconnected;
    }
    // Keeps status's validity, connection and result; takes position and
    // rotation from the last valid pose; zero velocity and prediction offset.
    static vr::DriverPose_t hold(const vr::DriverPose_t& status, const vr::DriverPose_t& last) {
        vr::DriverPose_t out = status;
        for (int i = 0; i < 3; ++i) {
            out.vecPosition[i] = last.vecPosition[i];
            out.vecVelocity[i] = out.vecAcceleration[i] = 0;
            out.vecAngularVelocity[i] = out.vecAngularAcceleration[i] = 0;
        }
        out.qRotation = last.qRotation;
        out.poseTimeOffset = 0;
        return out;
    }
    template<class Submit, class Request>
    void publish(const TargetReceiver& receiver, TargetReceiver::Time now, bool fresh,
                 const std::array<uint32_t, 8>& devices, Submit submit, Request request) {
        for (size_t slot = 0; slot < devices.size(); ++slot) {
            auto pose = receiver.pose(slot, now);
            const State state = classify(pose);
            if (state == State::valid) {
                if (!seen_valid_[slot]) { seen_valid_[slot] = true; request(slot); }
                held_[slot] = pose;
            } else if (seen_valid_[slot]) {
                pose = hold(pose, held_[slot]);
            }
            const uint32_t device = devices[slot];
            if (!seen_valid_[slot] || device == vr::k_unTrackedDeviceIndexInvalid) {
                device_[slot] = vr::k_unTrackedDeviceIndexInvalid; state_[slot] = State::unknown; continue;
            }
            if (device_[slot] != device) { device_[slot] = device; state_[slot] = State::unknown; }
            if (state == State::valid ? (fresh || state_[slot] != State::valid) : state != state_[slot])
                submit(slot, device, pose);
            state_[slot] = state;
        }
    }
    State state(size_t slot) const { return state_[slot]; }
    bool seen_valid(size_t slot) const { return seen_valid_[slot]; }
private:
    std::array<State, 8> state_{};
    std::array<bool, 8> seen_valid_{};
    std::array<vr::DriverPose_t, 8> held_{};
    std::array<uint32_t, 8> device_{vr::k_unTrackedDeviceIndexInvalid, vr::k_unTrackedDeviceIndexInvalid,
                                    vr::k_unTrackedDeviceIndexInvalid, vr::k_unTrackedDeviceIndexInvalid,
                                    vr::k_unTrackedDeviceIndexInvalid, vr::k_unTrackedDeviceIndexInvalid,
                                    vr::k_unTrackedDeviceIndexInvalid, vr::k_unTrackedDeviceIndexInvalid};
};

// Hands slot registration from the receive thread to SteamVR's main thread.
// request() may be called from any thread (atomic pending mask). add_pending()
// is called only from IServerTrackedDeviceProvider::RunFrame, so
// TrackedDeviceAdded runs on vrserver's main loop thread. Each requested slot is
// offered to add() at most once per Init; a rejected slot is not retried (the
// usual causes, such as activateMultipleDrivers=false, do not change at runtime).
class SlotRegistry {
public:
    void request(size_t slot) {
        if (slot < 8) pending_.fetch_or(1u << slot, std::memory_order_acq_rel);
    }
    template<class Add>
    void add_pending(Add add) {
        const uint32_t ready = pending_.load(std::memory_order_acquire) & ~attempted_;
        for (size_t slot = 0; slot < 8; ++slot) {
            if (!(ready & (1u << slot))) continue;
            attempted_ |= 1u << slot;
            add(slot);
        }
    }
    // Only while no thread can call request() (Init, before the receive thread starts).
    void reset() { pending_.store(0, std::memory_order_relaxed); attempted_ = 0; }
    bool attempted(size_t slot) const { return slot < 8 && (attempted_ & (1u << slot)); }
private:
    std::atomic<uint32_t> pending_{0};
    uint32_t attempted_ = 0;  // main thread only
};
