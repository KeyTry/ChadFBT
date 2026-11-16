#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include "openvr_driver.h"
#include "receiver.h"
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <system_error>
#include <thread>

static const char* names[] = {"hip","chest","left_foot","right_foot","left_knee","right_knee","left_elbow","right_elbow"};
class Tracker final : public vr::ITrackedDeviceServerDriver {
 // Written by SteamVR's activation calls, read by the receive thread.
 std::atomic<uint32_t> id{vr::k_unTrackedDeviceIndexInvalid}; std::string serial;
public:
 explicit Tracker(const char* name):serial(std::string("chadfbt_")+name) {}
 const std::string& Serial() const {return serial;}
 uint32_t Id() const {return id.load(std::memory_order_acquire);}
 vr::EVRInitError Activate(uint32_t object) override {
  auto p=vr::VRProperties(); auto c=p->TrackedDeviceToPropertyContainer(object);
  p->SetStringProperty(c,vr::Prop_TrackingSystemName_String,"chadfbt"); p->SetStringProperty(c,vr::Prop_ManufacturerName_String,"ChadFBT");
  p->SetStringProperty(c,vr::Prop_ModelNumber_String,"ChadFBT Tracker"); p->SetStringProperty(c,vr::Prop_SerialNumber_String,serial.c_str());
  p->SetStringProperty(c,vr::Prop_RegisteredDeviceType_String,(std::string("chadfbt/")+serial).c_str());
  // controller_type must match driver/chadfbt/resources/input/chadfbt_tracker_profile.json.
  p->SetStringProperty(c,vr::Prop_ControllerType_String,"chadfbt_tracker");
  p->SetStringProperty(c,vr::Prop_InputProfilePath_String,"{chadfbt}/input/chadfbt_tracker_profile.json");
  p->SetStringProperty(c,vr::Prop_RenderModelName_String,"{htc}/rendermodels/vr_tracker_vive_1_0");
  id.store(object,std::memory_order_release); return vr::VRInitError_None; }
 void Deactivate() override {id.store(vr::k_unTrackedDeviceIndexInvalid,std::memory_order_release);}
 void EnterStandby() override {} void* GetComponent(const char*) override {return nullptr;}
 void DebugRequest(const char*,char* response,uint32_t size) override {if(size)response[0]=0;}
 vr::DriverPose_t GetPose() override { vr::DriverPose_t pose{}; pose.qWorldFromDriverRotation.w=1; pose.qDriverFromHeadRotation.w=1; pose.qRotation.w=1; pose.result=vr::TrackingResult_Uninitialized; return pose; }
};

class Provider final: public vr::IServerTrackedDeviceProvider {
 // Tracker objects live from Init to Cleanup (TrackedDeviceAdded requires the
 // pointer to stay valid until Cleanup), but a slot's device is added to SteamVR
 // only after its first valid pose (SlotRegistry, RunFrame).
 std::array<std::unique_ptr<Tracker>,8> trackers; SlotRegistry registry; SOCKET sock=INVALID_SOCKET; std::thread receiver_thread; std::atomic<bool> stop{false}; bool wsa=false;
 void fail_cleanup() { if(sock!=INVALID_SOCKET){closesocket(sock);sock=INVALID_SOCKET;} if(wsa){WSACleanup();wsa=false;} for(auto& t:trackers)t.reset(); }
 // Owns all TargetReceiver state. Submits each newly accepted frame immediately,
 // and invalid states once per transition; RunFrame never submits poses. A
 // slot's first valid pose only requests registration; RunFrame performs it.
 void receive_loop() {
  TargetReceiver receiver(TargetReceiver::Clock::now()); PosePublisher publisher;
  const auto read=[this]() {
   TargetRead result{}; sockaddr_in from{}; int from_len=sizeof(from);
   const int count=recvfrom(sock,(char*)&result.frame,sizeof(result.frame),0,(sockaddr*)&from,&from_len);
   if(count==SOCKET_ERROR) {
    const int error=WSAGetLastError();
    result.status=(error==WSAEMSGSIZE || error==WSAECONNRESET) ? TargetReadStatus::discard :
                  error==WSAEWOULDBLOCK ? TargetReadStatus::empty : TargetReadStatus::error;
   } else {
    result.status=TargetReadStatus::packet; result.size=static_cast<size_t>(count);
    result.loopback=from.sin_family==AF_INET && from.sin_addr.s_addr==htonl(INADDR_LOOPBACK);
   }
   return result;
  };
  const auto submit=[](size_t,uint32_t device,const vr::DriverPose_t& pose) {
   vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device,pose,sizeof(pose));
  };
  const auto request=[this](size_t slot) { registry.request(slot); };
  while(!stop.load(std::memory_order_acquire)) {
   WSAPOLLFD fd{}; fd.fd=sock; fd.events=POLLRDNORM;
   const int ready=WSAPoll(&fd,1,5);  // wake on data, otherwise check expiry every few ms
   if(stop.load(std::memory_order_acquire)) break;
   bool fresh=false;
   if(ready==SOCKET_ERROR) { receiver.overflow_or_error(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
   else fresh=receiver.poll(TargetReceiver::Clock::now(),read);
   std::array<uint32_t,8> devices{};
   for(size_t i=0;i<trackers.size();++i) devices[i]=trackers[i]->Id();
   publisher.publish(receiver,TargetReceiver::Clock::now(),fresh,devices,submit,request);
  }
 }
public:
 vr::EVRInitError Init(vr::IVRDriverContext* context) override {VR_INIT_SERVER_DRIVER_CONTEXT(context);
  WSADATA w{}; if(WSAStartup(MAKEWORD(2,2),&w)!=0) return vr::VRInitError_Driver_Failed; wsa=true;
  sock=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP); if(sock==INVALID_SOCKET){fail_cleanup();return vr::VRInitError_Driver_Failed;}
  sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);addr.sin_port=htons(39570);
  if(bind(sock,(sockaddr*)&addr,sizeof(addr))==SOCKET_ERROR){vr::VRDriverLog()->Log("ChadFBT: cannot bind 127.0.0.1:39570");fail_cleanup();return vr::VRInitError_Driver_Failed;}
  u_long nonblocking=1; if(ioctlsocket(sock,FIONBIO,&nonblocking)==SOCKET_ERROR){vr::VRDriverLog()->Log("ChadFBT: cannot make socket nonblocking");fail_cleanup();return vr::VRInitError_Driver_Failed;}
  for(size_t i=0;i<trackers.size();++i) trackers[i]=std::make_unique<Tracker>(names[i]);
  registry.reset(); stop.store(false);
  try { receiver_thread=std::thread(&Provider::receive_loop,this); }
  catch(const std::system_error&) { vr::VRDriverLog()->Log("ChadFBT: cannot start receive thread"); fail_cleanup(); return vr::VRInitError_Driver_Failed; }
  // No TrackedDeviceAdded here: unused slots must never appear in SteamVR.
  return vr::VRInitError_None; }
 void Cleanup() override {
  stop.store(true,std::memory_order_release);
  if(sock!=INVALID_SOCKET){closesocket(sock);sock=INVALID_SOCKET;}
  if(receiver_thread.joinable()) receiver_thread.join();
  if(wsa){WSACleanup();wsa=false;}
  for(auto& t:trackers)t.reset();
  VR_CLEANUP_SERVER_DRIVER_CONTEXT();}
 const char* const* GetInterfaceVersions() override {return vr::k_InterfaceVersions;}
 // vrserver's main loop. TrackedDeviceAdded only queues activation (Valve:
 // "the device has been queued to activate"), so it does not block here.
 void RunFrame() override {
  registry.add_pending([this](size_t slot) {
   Tracker* t=trackers[slot].get(); if(!t) return;
   const bool added=vr::VRServerDriverHost()->TrackedDeviceAdded(t->Serial().c_str(),vr::TrackedDeviceClass_GenericTracker,t);
   vr::VRDriverLog()->Log((std::string("ChadFBT: ")+(added?"added ":"registration rejected for ")+t->Serial()).c_str());
  });
  vr::VREvent_t e{}; while(vr::VRServerDriverHost()->PollNextEvent(&e,sizeof(e))){} }
 bool ShouldBlockStandbyMode() override{return false;} void EnterStandby() override{} void LeaveStandby() override{}
};
static Provider provider;
extern "C" __declspec(dllexport) void* HmdDriverFactory(const char* name,int* error) {if(name && !strcmp(name,vr::IServerTrackedDeviceProvider_Version))return &provider; if(error)*error=vr::VRInitError_Init_InterfaceNotFound;return nullptr;}
