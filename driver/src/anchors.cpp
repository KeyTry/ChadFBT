#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <tlhelp32.h>
#include "openvr.h"
#include "wire.h"
#include "tracker_roles.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <timeapi.h>
#pragma comment(lib,"winmm.lib")
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif
// std::this_thread::sleep_for(10ms) rounds up to the ~15.6 ms system tick
// (measured 64 Hz). Pace with a high-resolution waitable timer (Windows 10
// 1803+), falling back to a 1 ms timer period with Sleep.
class Pacer {
 HANDLE timer=nullptr; bool raised_period=false;
public:
 Pacer() {
  timer=CreateWaitableTimerExW(nullptr,nullptr,CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,TIMER_ALL_ACCESS);
  if(!timer) raised_period=timeBeginPeriod(1)==TIMERR_NOERROR;
 }
 ~Pacer() { if(timer)CloseHandle(timer); if(raised_period)timeEndPeriod(1); }
 Pacer(const Pacer&)=delete; Pacer& operator=(const Pacer&)=delete;
 bool high_resolution() const {return timer!=nullptr;}
 void wait_until(std::chrono::steady_clock::time_point deadline) {
  const auto remaining=deadline-std::chrono::steady_clock::now();
  if(remaining<=std::chrono::steady_clock::duration::zero()) return;
  if(timer) {
   LARGE_INTEGER due{}; due.QuadPart=-(std::max<LONGLONG>)(1,std::chrono::duration_cast<std::chrono::nanoseconds>(remaining).count()/100);
   if(SetWaitableTimer(timer,&due,0,nullptr,nullptr,FALSE) && WaitForSingleObject(timer,INFINITE)==WAIT_OBJECT_0) return;
  }
  Sleep(static_cast<DWORD>((std::max<long long>)(1,std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count())));
 }
};
static constexpr auto anchor_period=std::chrono::microseconds(5000); // ~200 Hz
// SteamVR velocities that are nonfinite or out of protocol range are sent as
// zero; the pose itself stays valid (the receiver would otherwise drop the frame).
static void copy_velocity(const vr::HmdVector3_t& in,float max,float* out) {
 float v[3]={in.v[0],in.v[1],in.v[2]};
 if(bounded_vector(v,max)) std::memcpy(out,v,sizeof(v)); else std::memset(out,0,sizeof(v));
}
// Background mode must never start SteamVR. Check first to give the user a clear
// status; VRApplication_Background independently asks OpenVR not to start it.
static bool steamvr_running() {
 HANDLE snapshot=CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS,0);
 if(snapshot==INVALID_HANDLE_VALUE)return false;
 PROCESSENTRY32W entry{};entry.dwSize=sizeof(entry);bool found=false;
 if(Process32FirstW(snapshot,&entry)) do {if(_wcsicmp(entry.szExeFile,L"vrserver.exe")==0){found=true;break;}} while(Process32NextW(snapshot,&entry));
 CloseHandle(snapshot);return found;
}
static std::atomic<bool> running{true};
BOOL WINAPI signal_handler(DWORD) {running=false;return TRUE;}
// Use only known, constant serials in the JSON, so no runtime strings require escaping.
// Serial suffixes identify driver slots; SteamVR user-assigned roles are separate.
static int inspect(vr::IVRSystem* sys) {
 const char* slots[]={"hip","chest","left_foot","right_foot","left_knee","right_knee","left_elbow","right_elbow"};
 vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
 sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding,0,poses,vr::k_unMaxTrackedDeviceCount);
 const auto& hmd=poses[vr::k_unTrackedDeviceIndex_Hmd];
 float hmd_matrix[12];for(int i=0;i<3;++i)for(int j=0;j<4;++j)hmd_matrix[i*4+j]=hmd.mDeviceToAbsoluteTracking.m[i][j];
 const bool hmd_valid=hmd.bDeviceIsConnected && hmd.bPoseIsValid && rigid(hmd_matrix);
 std::printf("{\"space\":\"standing\",\"hmd\":{\"connected\":%s,\"pose_valid\":%s,\"height_m\":",hmd.bDeviceIsConnected?"true":"false",hmd_valid?"true":"false");
 if(hmd_valid)std::printf("%.9g",static_cast<double>(hmd_matrix[7]));else std::printf("null");
 std::printf("},\"trackers\":[");
 for(int slot=0;slot<8;++slot){
  char expected[64];std::snprintf(expected,sizeof(expected),"chadfbt_%s",slots[slot]);
  uint32_t index=vr::k_unTrackedDeviceIndexInvalid;
  for(uint32_t i=0;i<vr::k_unMaxTrackedDeviceCount;++i){
   char serial[128]{};vr::ETrackedPropertyError err=vr::TrackedProp_Success;
   sys->GetStringTrackedDeviceProperty(i,vr::Prop_SerialNumber_String,serial,sizeof(serial),&err);
   if(err==vr::TrackedProp_Success && std::strcmp(serial,expected)==0){index=i;break;}
  }
  if(slot)std::printf(",");
  std::printf("{\"slot\":\"%s\",\"serial\":\"%s\",\"present\":%s",slots[slot],expected,index<vr::k_unMaxTrackedDeviceCount?"true":"false");
  if(index<vr::k_unMaxTrackedDeviceCount){
   const auto& pose=poses[index];float mat[12];for(int i=0;i<3;++i)for(int j=0;j<4;++j)mat[i*4+j]=pose.mDeviceToAbsoluteTracking.m[i][j];
   const bool valid=pose.bDeviceIsConnected && pose.bPoseIsValid && rigid(mat);
   std::printf(",\"index\":%u,\"connected\":%s,\"pose_valid\":%s,\"tracking_result\":%d,\"position_m\":",index,pose.bDeviceIsConnected?"true":"false",valid?"true":"false",static_cast<int>(pose.eTrackingResult));
   if(valid)std::printf("[%.9g,%.9g,%.9g]",static_cast<double>(mat[3]),static_cast<double>(mat[7]),static_cast<double>(mat[11]));else std::printf("null");
  }
  std::printf("}");
 }
 std::printf("]}\n");return 0;
}
static int assign_core_roles(vr::IVRSettings& settings) {
 const auto results=chadfbt::assign_core_tracker_roles(settings);
 unsigned verified=0;
 for(std::size_t i=0;i<results.size();++i){
  const auto& request=chadfbt::core_tracker_roles[i];const auto& result=results[i];
  if(result.verified(request)){
   ++verified;std::printf("Verified %s = %s\n",request.key,request.role);
  }else{
   std::fprintf(stderr,"FAILED %s -> %s: write=%s (%d), read=%s (%d), read-back='%s'\n",
       request.key,request.role,settings.GetSettingsErrorNameFromEnum(result.write_error),static_cast<int>(result.write_error),
       settings.GetSettingsErrorNameFromEnum(result.read_error),static_cast<int>(result.read_error),result.read_value);
  }
 }
 std::printf("Verified %u of 6 requested ChadFBT core tracker roles.\n",verified);
 if(verified!=6)std::fprintf(stderr,"Role writes are independent; successful writes remain applied. Review the failures before retrying.\n");
 return verified==6?0:1;
}
int main(int argc,char** argv){ const bool inspect_only=argc==2 && std::strcmp(argv[1],"--inspect")==0; const bool assign_roles=argc==2 && std::strcmp(argv[1],"--assign-core-roles")==0; if(argc>1 && !inspect_only && !assign_roles){std::fprintf(stderr,"Usage: chadfbt-anchors.exe [--inspect | --assign-core-roles]\n");return 2;} SetConsoleCtrlHandler(signal_handler,TRUE); if(!steamvr_running()){std::fprintf(stderr,"SteamVR is not running; start SteamVR before the anchor helper.\n");return 1;} HMODULE dll=LoadLibraryW(L"openvr_api.dll"); if(!dll){std::fprintf(stderr,"openvr_api.dll not found next to helper\n");return 1;}
 using InitFn=uint32_t(__cdecl*)(vr::EVRInitError*,vr::EVRApplicationType,const char*); using GetFn=void*(__cdecl*)(const char*,vr::EVRInitError*); using ShutdownFn=void(__cdecl*)();
 auto init=(InitFn)GetProcAddress(dll,"VR_InitInternal2");auto get=(GetFn)GetProcAddress(dll,"VR_GetGenericInterface");auto shutdown=(ShutdownFn)GetProcAddress(dll,"VR_ShutdownInternal"); if(!init||!get||!shutdown){std::fprintf(stderr,"Invalid OpenVR runtime library\n");return 1;}
 vr::EVRInitError error=vr::VRInitError_None;init(&error,vr::VRApplication_Background,nullptr);if(error!=vr::VRInitError_None){std::fprintf(stderr,"OpenVR initialization failed: %d\n",error);return 1;}
 auto sys=(vr::IVRSystem*)get(vr::IVRSystem_Version,&error);if(!sys||error!=vr::VRInitError_None){std::fprintf(stderr,"OpenVR system unavailable: %d\n",error);shutdown();return 1;}
 if(inspect_only){int result=inspect(sys);shutdown();FreeLibrary(dll);return result;}
 if(assign_roles){
  auto settings=(vr::IVRSettings*)get(vr::IVRSettings_Version,&error);
  if(!settings||error!=vr::VRInitError_None){std::fprintf(stderr,"OpenVR settings unavailable: %d\n",error);shutdown();FreeLibrary(dll);return 1;}
  int result=assign_core_roles(*settings);shutdown();FreeLibrary(dll);return result;
 }
 WSADATA w{};if(WSAStartup(MAKEWORD(2,2),&w)){shutdown();return 1;} SOCKET s=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);if(s==INVALID_SOCKET){WSACleanup();shutdown();return 1;}
 sockaddr_in dest{};dest.sin_family=AF_INET;dest.sin_addr.s_addr=htonl(INADDR_LOOPBACK);dest.sin_port=htons(39571); uint64_t session=(GetTickCount64()<<16)^GetCurrentProcessId();if(!session)session=1;uint64_t sequence=0;
 Pacer pacer; if(!pacer.high_resolution())std::fprintf(stderr,"High-resolution timer unavailable; using 1 ms timer period.\n");
 auto next_send=std::chrono::steady_clock::now();
 while(running){vr::VREvent_t event{};while(sys->PollNextEvent(&event,sizeof(event))){if(event.eventType==vr::VREvent_Quit){sys->AcknowledgeQuit_Exiting();running=false;break;}}if(!running)break;AnchorFrame frame{};memcpy(frame.header.magic,"CFBA",4);frame.header.version=kWireVersion;frame.header.flags=0;frame.header.session=session;frame.header.sequence=++sequence;
 auto standing_from_raw=sys->GetRawZeroPoseToStandingAbsoluteTrackingPose();float* m=frame.header.raw_from_standing;
 float original[12];for(int i=0;i<3;++i)for(int j=0;j<4;++j)original[i*4+j]=standing_from_raw.m[i][j]; invert_rigid(original,m);
 vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding,0,poses,vr::k_unMaxTrackedDeviceCount);
 uint32_t indices[]={vr::k_unTrackedDeviceIndex_Hmd,sys->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand),sys->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand)};
 for(int i=0;i<3;++i){auto& out=frame.poses[i];out.q[0]=1;auto index=indices[i];if(index>=vr::k_unMaxTrackedDeviceCount||!poses[index].bPoseIsValid||!poses[index].bDeviceIsConnected||!rigid(m))continue;
 auto& mat=poses[index].mDeviceToAbsoluteTracking;float flat[12];for(int row=0;row<3;++row)for(int col=0;col<4;++col)flat[row*4+col]=mat.m[row][col]; if(!rigid(flat))continue;
 for(int row=0;row<3;++row)out.p[row]=flat[row*4+3];double q[4];quat_from_matrix(flat,q);for(int j=0;j<4;++j)out.q[j]=(float)q[j];
 // Standing-space velocities from the same TrackedDevicePose_t (vAngularVelocity frame: see PROTOCOL.md).
 copy_velocity(poses[index].vVelocity,kMaxLinearSpeed,out.v);copy_velocity(poses[index].vAngularVelocity,kMaxAngularSpeed,out.w);
 out.valid=valid_pose(out)?1:0; if(!out.valid){out=WirePose{};out.q[0]=1;} }
 sendto(s,(const char*)&frame,sizeof(frame),0,(sockaddr*)&dest,sizeof(dest));
 // Fixed-rate schedule; after a stall, resynchronise instead of bursting.
 next_send+=anchor_period; const auto now=std::chrono::steady_clock::now();
 if(next_send<now-anchor_period)next_send=now; pacer.wait_until(next_send); }
 closesocket(s);WSACleanup();shutdown();FreeLibrary(dll);return 0; }



