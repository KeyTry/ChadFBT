#include "wire.h"
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#define CHECK(c) do { if (!(c)) { std::cerr << "FAILED " << __FILE__ << ":" << __LINE__ << " " #c << "\n"; std::exit(1); } } while (0)

static std::string read_all(const char* path) {
 std::ifstream f(path); CHECK(f.good());
 return std::string((std::istreambuf_iterator<char>(f)), {});
}
static float parse_float(const std::string& token) {
 if(token=="nan") return std::numeric_limits<float>::quiet_NaN();
 if(token=="inf") return std::numeric_limits<float>::infinity();
 if(token=="-inf") return -std::numeric_limits<float>::infinity();
 return std::stof(token);
}

// The same case table is run by steamvr-bridge's Pose::wire_valid test.
static void shared_pose_validity(const char* path) {
 std::istringstream lines(read_all(path)); std::string line; int checked=0;
 while(std::getline(lines,line)) {
  if(line.empty() || line[0]=='#' || line.find_first_not_of(" \t\r")==std::string::npos) continue;
  std::istringstream fields(line); std::vector<float> v; std::string token;
  while(fields>>token) v.push_back(parse_float(token));
  CHECK(v.size()==16);
  WirePose p{}; p.valid=static_cast<uint32_t>(v[0]);
  for(int i=0;i<3;++i){p.p[i]=v[1+i];p.v[i]=v[8+i];p.w[i]=v[11+i];}
  for(int i=0;i<4;++i)p.q[i]=v[4+i];
  if(valid_pose(p)!=(v[14]==1.f)) { std::cerr << "case: " << line << "\n"; CHECK(false); }
  ++checked;
 }
 CHECK(checked>=16);
}

// The helper fills AnchorFrame directly; the Rust decoder reads the same fixture.
static void anchor_fixture(const char* path) {
 std::string hex; for(char c:read_all(path)) if(!std::isspace(static_cast<unsigned char>(c))) hex.push_back(c);
 CHECK(hex.size()==sizeof(AnchorFrame)*2);
 std::vector<unsigned char> bytes; for(size_t i=0;i<hex.size();i+=2) bytes.push_back(static_cast<unsigned char>(std::stoul(hex.substr(i,2),nullptr,16)));
 AnchorFrame a{}; std::memcpy(&a,bytes.data(),sizeof(a));
 CHECK(!std::memcmp(a.header.magic,"CFBA",4) && a.header.version==kWireVersion && a.header.session==11 && a.header.sequence==12 && a.header.flags==0);
 CHECK(rigid(a.header.raw_from_standing) && a.header.raw_from_standing[3]==2.f);
 CHECK(a.poses[0].valid==1 && a.poses[0].p[1]==1.6f && a.poses[0].v[0]==0.5f && a.poses[0].w[1]==1.f);
 CHECK(a.poses[1].valid==0 && a.poses[1].q[0]==1.f);
 CHECK(a.poses[2].valid==1 && a.poses[2].q[3]==0.5f && a.poses[2].v[1]==-0.25f && a.poses[2].w[2]==2.f);
 for(const auto& p:a.poses) CHECK(valid_pose(p));
}

int main(int argc,char** argv) {
 CHECK(argc==3);
 float id[12]={1,0,0,2, 0,1,0,3, 0,0,1,4}; CHECK(rigid(id));
 float reflected[12]={-1,0,0,0, 0,1,0,0, 0,0,1,0}; CHECK(!rigid(reflected));
 float scaled[12]={2,0,0,0, 0,1,0,0, 0,0,1,0}; CHECK(!rigid(scaled));
 auto invalid=id[0];id[0]=std::numeric_limits<float>::quiet_NaN();CHECK(!rigid(id));id[0]=invalid;
 WirePoseV1 p{};p.valid=1;p.q[0]=1;CHECK(valid_pose(p));p.q[0]=2;CHECK(!valid_pose(p));p.valid=2;CHECK(!valid_pose(p));
 float transform[12]={0,0,1,2, 0,1,0,3, -1,0,0,4}; float inverse[12]{}; invert_rigid(transform,inverse); CHECK(rigid(inverse)); for(int i=0;i<3;++i){ for(int j=0;j<3;++j){float dot=0; for(int k=0;k<3;++k) dot+=inverse[i*4+k]*transform[k*4+j]; CHECK(std::fabs(dot-(i==j?1.f:0.f))<1e-6f); } float t=inverse[i*4+3];for(int k=0;k<3;++k)t+=inverse[i*4+k]*transform[k*4+3];CHECK(std::fabs(t)<1e-6f); }
 float yaw[12]={0,0,1,0, 0,1,0,0, -1,0,0,0};double q[4];CHECK(rigid(yaw));quat_from_matrix(yaw,q);CHECK(std::fabs(q[0]-std::sqrt(0.5))<1e-6);CHECK(std::fabs(q[2]-std::sqrt(0.5))<1e-6);
 // Velocity rotation uses only the 3x3 part: translation must not leak in.
 float v[3]={1,0,0}; double out[3]; rotate_vector(transform,v,out); CHECK(out[0]==0 && out[1]==0 && out[2]==-1);
 CHECK(valid_pose_age(0.f) && valid_pose_age(0.25f) && !valid_pose_age(0.2501f) && !valid_pose_age(-0.f-1e-9f) && !valid_pose_age(std::numeric_limits<float>::quiet_NaN()));
 shared_pose_validity(argv[1]);
 anchor_fixture(argv[2]);
 std::cout << "Wire layout, shared pose-validity table and anchor fixture tests passed.\n";
}
