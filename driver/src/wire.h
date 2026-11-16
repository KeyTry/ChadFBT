#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <algorithm>

// Wire protocol v2 (see driver/PROTOCOL.md). v1 layouts are kept only so the
// driver can still accept target frames from an older app build.
constexpr uint32_t kWireVersion=2;
constexpr uint32_t kFlagApplyAngularVelocity=1u;   // header flags bit 0 (targets only)
constexpr float kMaxPosition=1000.f;               // metres, per component
constexpr float kMaxLinearSpeed=50.f;              // m/s, vector magnitude
constexpr float kMaxAngularSpeed=100.f;            // rad/s, vector magnitude
constexpr float kMaxPoseAge=0.25f;                 // seconds

#pragma pack(push,1)
struct WirePoseV1 { uint32_t valid; float p[3]; float q[4]; };
struct WireHeaderV1 { char magic[4]; uint32_t version; uint64_t session, sequence; float raw_from_standing[12]; };
struct TargetFrameV1 { WireHeaderV1 header; WirePoseV1 poses[8]; };
struct AnchorFrameV1 { WireHeaderV1 header; WirePoseV1 poses[3]; };

// v2: standing-space linear velocity (m/s) and angular velocity (rad/s, axis-angle).
struct WirePose { uint32_t valid; float p[3]; float q[4]; float v[3]; float w[3]; };
struct WireHeader { char magic[4]; uint32_t version; uint64_t session, sequence; uint32_t flags; float raw_from_standing[12]; };
struct TargetFrame { WireHeader header; float pose_age_secs; WirePose poses[8]; };
struct AnchorFrame { WireHeader header; WirePose poses[3]; };
#pragma pack(pop)
static_assert(sizeof(TargetFrameV1)==328 && sizeof(AnchorFrameV1)==168,"v1 wire layout");
static_assert(sizeof(WirePose)==56 && sizeof(WireHeader)==76,"v2 record/header layout");
static_assert(sizeof(TargetFrame)==528 && sizeof(AnchorFrame)==244,"v2 wire layout");
static_assert(offsetof(WireHeader,session)==8 && offsetof(WireHeader,sequence)==16 &&
              offsetof(WireHeader,flags)==24 && offsetof(WireHeader,raw_from_standing)==28,"v2 header offsets");
static_assert(offsetof(TargetFrame,pose_age_secs)==76 && offsetof(TargetFrame,poses)==80 &&
              offsetof(AnchorFrame,poses)==76,"v2 frame offsets");
static_assert(offsetof(WirePose,q)==16 && offsetof(WirePose,v)==32 && offsetof(WirePose,w)==44,"v2 pose offsets");

inline bool rigid(const float* m) {
 for(int i=0;i<12;++i) if(!std::isfinite(m[i]) || std::fabs(m[i])>1000.f) return false;
 for(int i=0;i<3;++i) for(int j=0;j<3;++j) {float v=0; for(int k=0;k<3;++k)v+=m[i*4+k]*m[j*4+k]; if(std::fabs(v-(i==j?1.f:0.f))>0.01f)return false;}
 float d=m[0]*(m[5]*m[10]-m[6]*m[9])-m[1]*(m[4]*m[10]-m[6]*m[8])+m[2]*(m[4]*m[9]-m[5]*m[8]); return std::fabs(d-1.f)<0.01f;
}
// Finite vector whose magnitude does not exceed max. Mirrored by steamvr-bridge's Pose::wire_valid.
inline bool bounded_vector(const float* v,float max) {
 for(int i=0;i<3;++i) if(!std::isfinite(v[i])) return false;
 const float n=v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
 return n<=max*max;
}
inline bool valid_pose_fields(uint32_t valid,const float* p,const float* q) {
 if(valid>1) return false;
 for(int i=0;i<3;++i) if(!std::isfinite(p[i])||std::fabs(p[i])>kMaxPosition) return false;
 float n=0; for(int i=0;i<4;++i) {if(!std::isfinite(q[i]))return false;n+=q[i]*q[i];}
 return !valid || (n>0.99f&&n<1.01f);
}
// Validity rules apply to every record, including records marked invalid.
inline bool valid_pose(const WirePoseV1& p) { return valid_pose_fields(p.valid,p.p,p.q); }
inline bool valid_pose(const WirePose& p) {
 return valid_pose_fields(p.valid,p.p,p.q) && bounded_vector(p.v,kMaxLinearSpeed) && bounded_vector(p.w,kMaxAngularSpeed);
}
inline bool valid_pose_age(float age) { return std::isfinite(age) && age>=0.f && age<=kMaxPoseAge; }

// A v1 target frame is the v2 frame with zero velocities, zero flags and zero pose age.
inline void upgrade_v1(const TargetFrameV1& in,TargetFrame& out) {
 out=TargetFrame{};
 std::memcpy(out.header.magic,in.header.magic,4);
 out.header.version=in.header.version; out.header.session=in.header.session; out.header.sequence=in.header.sequence;
 std::memcpy(out.header.raw_from_standing,in.header.raw_from_standing,sizeof(out.header.raw_from_standing));
 for(int i=0;i<8;++i){out.poses[i].valid=in.poses[i].valid; std::memcpy(out.poses[i].p,in.poses[i].p,sizeof(in.poses[i].p)); std::memcpy(out.poses[i].q,in.poses[i].q,sizeof(in.poses[i].q));}
}

inline void quat_from_matrix(const float* m,double q[4]) { // stable conversion of orthonormal row-major 3x4
 double trace=m[0]+m[5]+m[10];
 if(trace>0) { double s=std::sqrt(trace+1)*2; q[0]=s/4; q[1]=(m[9]-m[6])/s; q[2]=(m[2]-m[8])/s; q[3]=(m[4]-m[1])/s; }
 else if(m[0]>m[5]&&m[0]>m[10]) {double s=std::sqrt(1+m[0]-m[5]-m[10])*2;q[0]=(m[9]-m[6])/s;q[1]=s/4;q[2]=(m[1]+m[4])/s;q[3]=(m[2]+m[8])/s;}
 else if(m[5]>m[10]) {double s=std::sqrt(1+m[5]-m[0]-m[10])*2;q[0]=(m[2]-m[8])/s;q[1]=(m[1]+m[4])/s;q[2]=s/4;q[3]=(m[6]+m[9])/s;}
 else {double s=std::sqrt(1+m[10]-m[0]-m[5])*2;q[0]=(m[4]-m[1])/s;q[1]=(m[2]+m[8])/s;q[2]=(m[6]+m[9])/s;q[3]=s/4;}
}

// Applies only the rotation part of a row-major 3x4 transform (for velocities).
inline void rotate_vector(const float* m,const float* v,double out[3]) {
 for(int i=0;i<3;++i) out[i]=double(m[i*4])*v[0]+double(m[i*4+1])*v[1]+double(m[i*4+2])*v[2];
}

// Inverse of a rigid row-major 3x4 transform: R^-1 = R^T and t^-1 = -R^T t.
inline void invert_rigid(const float* input,float* output) {
 for(int i=0;i<3;++i){for(int j=0;j<3;++j)output[i*4+j]=input[j*4+i]; output[i*4+3]=0; for(int j=0;j<3;++j)output[i*4+3]-=output[i*4+j]*input[j*4+3];}
}
