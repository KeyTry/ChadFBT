#include "receiver.h"
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

#define CHECK(c) do { if (!(c)) { std::cerr << "FAILED " << __FILE__ << ':' << __LINE__ << " " #c << '\n'; std::exit(1); } } while (0)
static bool near(double a,double b) { return std::fabs(a-b) < 1e-5; }
static TargetReceiver::Time at(int ms) { return TargetReceiver::Time{} + std::chrono::milliseconds(ms); }
static std::vector<unsigned char> fixture(const char* path,size_t size) {
    std::ifstream f(path);
    CHECK(f.good());
    std::string text((std::istreambuf_iterator<char>(f)), {}), hex;
    for (char c : text) if (!std::isspace(static_cast<unsigned char>(c))) hex.push_back(c);
    CHECK(hex.size() == size*2);
    std::vector<unsigned char> result;
    for(size_t i=0;i<hex.size();i+=2) result.push_back(static_cast<unsigned char>(std::stoul(hex.substr(i,2),nullptr,16)));
    return result;
}
static bool zero(const double* v) { return v[0]==0 && v[1]==0 && v[2]==0; }
static bool out_of_range(const vr::DriverPose_t& p) {
    return p.deviceIsConnected && !p.poseIsValid && p.result==vr::TrackingResult_Running_OutOfRange;
}
static bool disconnected(const vr::DriverPose_t& p) { return !p.deviceIsConnected && !p.poseIsValid; }

int main(int argc,char** argv) {
    CHECK(argc==3);
    // argv[1]: legacy v1 fixture (older app builds); argv[2]: v2 fixture written by the Rust bridge.
    const auto legacy=fixture(argv[1],sizeof(TargetFrameV1));
    const auto bytes=fixture(argv[2],sizeof(TargetFrame));
    TargetFrame frame{}; std::memcpy(&frame,bytes.data(),sizeof(frame));
    CHECK(frame.header.version==2 && frame.header.session==0x1122334455667788ULL && frame.header.sequence==123);
    CHECK(frame.header.flags==kFlagApplyAngularVelocity && frame.pose_age_secs==0.015625f);

    // The Rust serializer is checked against these fixtures by its unit tests;
    // this is the real driver's receiver and pose conversion.
    {
        TargetReceiver r(at(0)); r.begin_poll(at(0));
        CHECK(r.accept(legacy.data(),legacy.size(),true,at(0)));
        auto hip=r.pose(0,at(0));
        CHECK(hip.poseIsValid && hip.deviceIsConnected && hip.result==vr::TrackingResult_Running_OK);
        CHECK(near(hip.vecPosition[0],5) && near(hip.vecPosition[1],5) && near(hip.vecPosition[2],3));
        CHECK(near(hip.qRotation.w,.5) && near(hip.qRotation.x,.5) && near(hip.qRotation.y,.5) && near(hip.qRotation.z,-.5));
        CHECK(zero(hip.vecVelocity) && zero(hip.vecAngularVelocity) && hip.poseTimeOffset==0); // v1 = zero velocity, zero age
        CHECK(out_of_range(r.pose(1,at(0))));
        // v1 records with a malformed field still reject the frame.
        auto bad=legacy; TargetFrameV1 v1{}; std::memcpy(&v1,bad.data(),sizeof(v1)); v1.header.sequence=124; v1.poses[1].p[0]=NAN;
        CHECK(!r.accept(&v1,sizeof(v1),true,at(1)));
        v1.poses[1].p[0]=0; v1.header.version=2; CHECK(!r.accept(&v1,sizeof(v1),true,at(1))); // v2 version, v1 size
    }

    TargetReceiver r(at(0)); r.begin_poll(at(0));
    CHECK(r.accept(bytes.data(),bytes.size(),true,at(0)));
    auto hip=r.pose(0,at(0));
    CHECK(hip.poseIsValid && hip.deviceIsConnected && hip.result==vr::TrackingResult_Running_OK);
    CHECK(near(hip.vecPosition[0],5) && near(hip.vecPosition[1],5) && near(hip.vecPosition[2],3));
    CHECK(near(hip.qRotation.w,.5) && near(hip.qRotation.x,.5) && near(hip.qRotation.y,.5) && near(hip.qRotation.z,-.5));
    CHECK(near(hip.qWorldFromDriverRotation.w,1) && near(hip.qDriverFromHeadRotation.w,1));
    // Velocities are rotated by the rotation part of raw_from_standing only (no translation).
    CHECK(near(hip.vecVelocity[0],2) && near(hip.vecVelocity[1],-1) && near(hip.vecVelocity[2],-.5));
    CHECK(near(hip.vecAngularVelocity[0],3) && near(hip.vecAngularVelocity[1],0) && near(hip.vecAngularVelocity[2],0));
    CHECK(near(hip.poseTimeOffset,-0.015625));
    CHECK(near(r.pose(0,at(20)).poseTimeOffset,-0.035625)); // age keeps growing after receipt
    CHECK(out_of_range(r.pose(1,at(0))));
    auto foot=r.pose(2,at(0));
    CHECK(foot.poseIsValid && near(foot.vecPosition[0],1.5) && near(foot.vecPosition[1],4.2) && near(foot.vecPosition[2],4.25));
    CHECK(near(foot.vecVelocity[0],-.25) && near(foot.vecVelocity[1],0) && near(foot.vecVelocity[2],0));
    CHECK(near(foot.vecAngularVelocity[0],0) && near(foot.vecAngularVelocity[1],0) && near(foot.vecAngularVelocity[2],-1));
    CHECK(out_of_range(r.pose(8,at(0))));

    // Angular velocity is opt-in (flag bit 0); linear velocity is always applied.
    {
        TargetReceiver plain(at(0)); auto f=frame; f.header.flags=0;
        CHECK(plain.accept(&f,sizeof(f),true,at(0)));
        auto p=plain.pose(0,at(0)); CHECK(zero(p.vecAngularVelocity) && near(p.vecVelocity[0],2));
    }

    // Invalid, non-local, replayed and out-of-order packets do not refresh age.
    CHECK(!r.accept(bytes.data(),bytes.size()-1,true,at(40)));
    CHECK(!r.accept(bytes.data(),bytes.size(),false,at(40)));
    CHECK(!r.accept(bytes.data(),bytes.size(),true,at(80)));
    auto bad=frame; bad.header.sequence=124; bad.poses[0].q[0]=NAN;
    CHECK(!r.accept(&bad,sizeof(bad),true,at(80)));
    bad=frame;bad.header.sequence=124;bad.header.raw_from_standing[0]=2;
    CHECK(!r.accept(&bad,sizeof(bad),true,at(80)));
    // v2 header and velocity validation rejects the whole frame.
    bad=frame;bad.header.sequence=124;bad.header.flags=2; CHECK(!r.accept(&bad,sizeof(bad),true,at(80)));
    bad=frame;bad.header.sequence=124;bad.header.flags=0x80000001u; CHECK(!r.accept(&bad,sizeof(bad),true,at(80)));
    bad=frame;bad.header.sequence=124;bad.header.version=3; CHECK(!r.accept(&bad,sizeof(bad),true,at(80)));
    for(float age : {-0.001f,0.2501f,NAN,INFINITY}) { bad=frame;bad.header.sequence=124;bad.pose_age_secs=age; CHECK(!r.accept(&bad,sizeof(bad),true,at(80))); }
    bad=frame;bad.header.sequence=124;bad.poses[5].v[1]=50.01f; CHECK(!r.accept(&bad,sizeof(bad),true,at(80))); // invalid slot still checked
    bad=frame;bad.header.sequence=124;bad.poses[0].v[2]=NAN; CHECK(!r.accept(&bad,sizeof(bad),true,at(80)));
    bad=frame;bad.header.sequence=124;bad.poses[0].w[0]=80;bad.poses[0].w[1]=60.1f; CHECK(!r.accept(&bad,sizeof(bad),true,at(80)));
    bad=frame;bad.header.sequence=124;bad.poses[0].w[0]=INFINITY; CHECK(!r.accept(&bad,sizeof(bad),true,at(80)));
    CHECK(r.pose(0,at(99)).poseIsValid);
    // Lease expiry (100 ms): the session is still alive, so the slot is out of range, not disconnected.
    CHECK(out_of_range(r.pose(0,at(100))));
    CHECK(out_of_range(r.pose(0,at(1000))));
    CHECK(disconnected(r.pose(0,at(1001))) && disconnected(r.pose(1,at(1001))));
    CHECK(disconnected(TargetReceiver(at(0)).pose(0,at(0)))); // nothing heard yet

    // Session takeover is gated; exact expiry permits a new sender. Poll at
    // normal intervals so this tests takeover rather than pause draining.
    TargetReceiver session(at(0)); CHECK(session.accept(&frame,sizeof(frame),true,at(0)));
    auto next=frame; next.header.session=2;next.header.sequence=1;
    CHECK(!session.accept(&next,sizeof(next),true,at(80)));
    session.begin_poll(at(40));session.begin_poll(at(80));session.begin_poll(at(100));
    CHECK(session.accept(&next,sizeof(next),true,at(100)));
    CHECK(session.pose(0,at(100)).poseIsValid);
    next.header.sequence=1; CHECK(!session.accept(&next,sizeof(next),true,at(104)));
    next.header.sequence=2; CHECK(session.accept(&next,sizeof(next),true,at(108)));
    session.begin_poll(at(140));session.begin_poll(at(180));session.begin_poll(at(210));
    auto retired=frame;retired.header.sequence=999;
    CHECK(!session.accept(&retired,sizeof(retired),true,at(210)));
    next.header.session=3;next.header.sequence=1;CHECK(session.accept(&next,sizeof(next),true,at(210)));

    // If the consumer pauses, queued data is discarded to empty, then only a
    // later datagram may revive poses. Budget exhaustion behaves identically.
    TargetReceiver pause(at(0)); CHECK(pause.accept(&frame,sizeof(frame),true,at(0)));
    const auto old=frame;
    pause.begin_poll(at(100)); CHECK(pause.flushing() && out_of_range(pause.pose(0,at(100))));
    frame.header.sequence++; CHECK(!pause.accept(&frame,sizeof(frame),true,at(101)));
    pause.observe_discarded(&frame,sizeof(frame),true);
    pause.drained(); CHECK(!pause.pose(0,at(101)).poseIsValid);
    CHECK(!pause.accept(&old,sizeof(old),true,at(101))); // watermark survived invalidation
    CHECK(!pause.accept(&frame,sizeof(frame),true,at(101))); // observed during drain
    frame.header.sequence++; CHECK(pause.accept(&frame,sizeof(frame),true,at(102)));
    pause.overflow_or_error(); CHECK(!pause.pose(0,at(102)).poseIsValid);
    frame.header.sequence++; CHECK(!pause.accept(&frame,sizeof(frame),true,at(103)));
    pause.drained(); CHECK(pause.accept(&frame,sizeof(frame),true,at(104)));
    CHECK(!pause.pose(0,at(103)).poseIsValid); // receiver clock cannot go backwards

    // Exercise the same 64-read pump used by the receive thread, without sockets.
    TargetReceiver budget(at(0)); auto pending=frame;pending.header.sequence=200;
    int count=0;
    CHECK(!budget.poll(at(0),[&] { TargetRead item{};item.status=TargetReadStatus::packet;
        item.frame=pending;item.frame.header.sequence+=static_cast<uint64_t>(count++);item.size=sizeof(item.frame);item.loopback=true;return item; }));
    CHECK(count==64 && budget.flushing() && !budget.pose(0,at(0)).poseIsValid);
    count=0;
    CHECK(!budget.poll(at(10),[&] { TargetRead item{};if(count++<2) {item.status=TargetReadStatus::packet;item.frame=pending;item.frame.header.sequence+=100;item.size=sizeof(item.frame);item.loopback=true;} return item; }));
    CHECK(!budget.flushing() && !budget.pose(0,at(10)).poseIsValid);
    count=0;
    CHECK(budget.poll(at(20),[&] { TargetRead item{};if(count++==0) {item.status=TargetReadStatus::packet;item.frame=pending;item.frame.header.sequence+=101;item.size=sizeof(item.frame);item.loopback=true;} return item; }));
    CHECK(budget.pose(0,at(20)).poseIsValid);
    count=0;
    CHECK(!budget.poll(at(25),[&] { count++; return TargetRead{}; })); // nothing new
    CHECK(!budget.poll(at(26),[&] { TargetRead item{}; if(count++==1) item.status=TargetReadStatus::error; else item.status=TargetReadStatus::discard; return item; }));
    CHECK(budget.flushing());

    // Publisher and lazy registration. The fixture drives slots 0 (hip) and 2
    // (left foot) only, like an app without upper-arm IMUs; the other six slots
    // must never request a device or submit a pose.
    {
        TargetReceiver rx(at(0)); PosePublisher pub; SlotRegistry reg;
        std::array<uint32_t,8> ids{}; ids.fill(vr::k_unTrackedDeviceIndexInvalid); // nothing added at Init
        std::vector<std::pair<size_t,vr::DriverPose_t>> sent; std::vector<size_t> requested, added;
        auto submit=[&](size_t slot,uint32_t device,const vr::DriverPose_t& p) { CHECK(device==ids[slot] && device!=vr::k_unTrackedDeviceIndexInvalid); sent.emplace_back(slot,p); };
        auto request=[&](size_t slot) { requested.push_back(slot); reg.request(slot); };
        auto run_frame=[&] { reg.add_pending([&](size_t slot) { added.push_back(slot); }); };
        auto publish=[&](int ms,bool fresh) { sent.clear(); pub.publish(rx,at(ms),fresh,ids,submit,request); run_frame(); };
        auto only=[&](std::initializer_list<size_t> slots) {
            if(sent.size()!=slots.size()) return false;
            size_t i=0; for(size_t s:slots) if(sent[i++].first!=s) return false; return true;
        };
        auto at_hip=[&](const vr::DriverPose_t& p,double x) { // hip held/live position and rotation
            return near(p.vecPosition[0],x) && near(p.vecPosition[1],5) && near(p.vecPosition[2],3) &&
                   near(p.qRotation.w,.5) && near(p.qRotation.x,.5) && near(p.qRotation.y,.5) && near(p.qRotation.z,-.5);
        };
        auto still=[&](const vr::DriverPose_t& p) { return zero(p.vecVelocity) && zero(p.vecAngularVelocity) && p.poseTimeOffset==0; };

        publish(0,false); publish(5,false); // nothing heard: nothing requested, nothing submitted
        CHECK(sent.empty() && requested.empty() && added.empty());

        // An all-invalid frame (e.g. output off) still requests nothing.
        auto f=frame; f.header.session=9; f.header.sequence=1;
        auto none=f; for(auto& p:none.poses) { p=WirePose{}; p.q[0]=1; }
        CHECK(rx.accept(&none,sizeof(none),true,at(8))); publish(8,true);
        CHECK(sent.empty() && requested.empty() && added.empty());

        // First valid frame: exactly the valid slots request registration; nothing is
        // submitted before SteamVR activates the device.
        f.header.sequence=2; CHECK(rx.accept(&f,sizeof(f),true,at(10))); publish(10,true);
        CHECK(sent.empty() && requested==std::vector<size_t>({0,2}) && added==std::vector<size_t>({0,2}));
        publish(12,false); CHECK(added.size()==2); // RunFrame never adds a slot twice

        // Activation (hip only for now) gets the current valid state once; no re-stamping.
        ids[0]=10; publish(15,false);
        CHECK(only({0}) && sent[0].second.poseIsValid && at_hip(sent[0].second,5));
        publish(16,false); CHECK(sent.empty());
        f.header.sequence=3; CHECK(rx.accept(&f,sizeof(f),true,at(20))); publish(20,true);
        CHECK(only({0}) && sent[0].second.poseIsValid);

        // Invalid after valid while the app is still sending: connected, out of range,
        // last valid position/rotation, zero velocity; once per transition.
        auto lost=f; lost.header.sequence=4; lost.poses[0]=WirePose{}; lost.poses[0].q[0]=1;
        CHECK(rx.accept(&lost,sizeof(lost),true,at(30))); publish(30,true);
        CHECK(only({0}) && out_of_range(sent[0].second) && at_hip(sent[0].second,5) && still(sent[0].second));
        publish(35,false); CHECK(sent.empty());
        lost.header.sequence=5; CHECK(rx.accept(&lost,sizeof(lost),true,at(40))); publish(40,true); CHECK(sent.empty());

        // Late activation of the left foot: its current (valid) state, once.
        ids[2]=12; publish(45,false);
        CHECK(only({2}) && sent[0].second.poseIsValid && near(sent[0].second.vecPosition[0],1.5));

        // Re-validation at a new position updates what is held afterwards.
        auto moved=f; moved.header.sequence=6; moved.poses[0].p[2]+=1; // hip raw x = standing z + 2 = 6
        CHECK(rx.accept(&moved,sizeof(moved),true,at(50))); publish(50,true);
        CHECK(only({0,2}) && sent[0].second.poseIsValid && at_hip(sent[0].second,6));

        // Lease expiry: both registered slots go out of range once, holding their last pose.
        publish(149,false); CHECK(sent.empty());
        publish(150,false);
        CHECK(only({0,2}) && out_of_range(sent[0].second) && at_hip(sent[0].second,6) && still(sent[0].second));
        CHECK(out_of_range(sent[1].second) && near(sent[1].second.vecPosition[0],1.5) && near(sent[1].second.vecPosition[1],4.2));
        publish(500,false); CHECK(sent.empty());

        // Deactivate and reactivate (new index): current state is resubmitted.
        ids[0]=vr::k_unTrackedDeviceIndexInvalid; publish(600,false); CHECK(sent.empty());
        ids[0]=20; publish(610,false); CHECK(only({0}) && out_of_range(sent[0].second) && at_hip(sent[0].second,6));

        // No frame for > 1 s (app closed): disconnected once, still held, never zeros.
        publish(1050,false); CHECK(sent.empty());
        publish(1051,false);
        CHECK(only({0,2}) && disconnected(sent[0].second) && at_hip(sent[0].second,6) && still(sent[0].second));
        CHECK(disconnected(sent[1].second) && near(sent[1].second.vecPosition[0],1.5));
        publish(1100,false); CHECK(sent.empty());

        // The app returns: registered slots become valid again without re-registration.
        f.header.sequence=7; rx.begin_poll(at(1200)); rx.drained(); CHECK(rx.accept(&f,sizeof(f),true,at(1200)));
        publish(1200,true); CHECK(only({0,2}) && sent[0].second.poseIsValid && sent[1].second.poseIsValid);

        // A previously driven slot marked permanently unused simply stays invalid (held).
        auto unused=f; unused.header.sequence=8; unused.poses[2]=WirePose{}; unused.poses[2].q[0]=1;
        for(int t=1210;t<=1250;t+=10) { unused.header.sequence++; CHECK(rx.accept(&unused,sizeof(unused),true,at(t))); publish(t,true);
            CHECK(sent.size()>=1 && sent[0].first==0 && sent[0].second.poseIsValid);
            if(t==1210) CHECK(sent.size()==2 && out_of_range(sent[1].second) && near(sent[1].second.vecPosition[0],1.5));
            else CHECK(sent.size()==1); }

        // Even with a device index, a slot that never had a valid pose submits nothing.
        ids[6]=16; publish(1260,false); CHECK(sent.empty()); publish(2500,false); CHECK(only({0,2})); ids[6]=vr::k_unTrackedDeviceIndexInvalid;

        // Unused slots never asked for a device and never submitted anything.
        CHECK(requested==std::vector<size_t>({0,2}) && added==std::vector<size_t>({0,2}));
        for(size_t s:{1,3,4,5,6,7}) CHECK(!pub.seen_valid(s) && !reg.attempted(s) && pub.state(s)==PosePublisher::State::unknown);
    }

    // Legacy v1 frames drive lazy registration identically (zero velocity).
    {
        TargetReceiver rx(at(0)); PosePublisher pub; SlotRegistry reg;
        std::array<uint32_t,8> ids{}; ids.fill(vr::k_unTrackedDeviceIndexInvalid);
        std::vector<std::pair<size_t,vr::DriverPose_t>> sent; std::vector<size_t> added;
        auto submit=[&](size_t slot,uint32_t,const vr::DriverPose_t& p) { sent.emplace_back(slot,p); };
        auto request=[&](size_t slot) { reg.request(slot); };
        CHECK(rx.accept(legacy.data(),legacy.size(),true,at(0)));
        pub.publish(rx,at(0),true,ids,submit,request); reg.add_pending([&](size_t s) { added.push_back(s); });
        CHECK(sent.empty() && added==std::vector<size_t>({0,2}));
        ids[0]=3; pub.publish(rx,at(1),false,ids,submit,request);
        CHECK(sent.size()==1 && sent[0].second.poseIsValid && zero(sent[0].second.vecVelocity));
        TargetFrameV1 v1{}; std::memcpy(&v1,legacy.data(),sizeof(v1)); v1.header.sequence++; v1.poses[0]=WirePoseV1{}; v1.poses[0].q[0]=1;
        CHECK(rx.accept(&v1,sizeof(v1),true,at(10))); sent.clear(); pub.publish(rx,at(10),true,ids,submit,request);
        CHECK(sent.size()==1 && out_of_range(sent[0].second) && near(sent[0].second.vecPosition[0],5) && near(sent[0].second.vecPosition[2],3));
    }

    // SlotRegistry: request() from another thread is seen by add_pending(); each slot is
    // offered once (a rejected add is not retried), and reset() starts a new Init.
    {
        SlotRegistry reg; std::vector<size_t> added;
        auto add=[&](size_t s) { added.push_back(s); };
        reg.add_pending(add); CHECK(added.empty());
        std::thread worker([&] { reg.request(4); reg.request(1); reg.request(4); reg.request(8); });
        worker.join();
        reg.add_pending(add); CHECK(added==std::vector<size_t>({1,4}));
        reg.request(1); reg.add_pending(add); CHECK(added.size()==2);
        reg.reset(); reg.add_pending(add); CHECK(added.size()==2 && !reg.attempted(1));
        reg.request(7); reg.add_pending(add); CHECK(added.back()==7 && reg.attempted(7));
    }
    std::cout << "Receiver v1/v2 fixture, velocity, pose age, invalidation, sequencing, drain, publish and lazy-registration tests passed.\n";
}
