#include "tracker_roles.h"
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#define CHECK(c) do { if (!(c)) { std::cerr << "FAILED " << __FILE__ << ":" << __LINE__ << " " #c << "\n"; std::exit(1); } } while (0)

class FakeSettings final : public vr::IVRSettings {
public:
 std::map<std::string,std::string> values;
 std::string failed_write,failed_read,mismatched_read;
 std::vector<std::string> writes,reads;
 const char* GetSettingsErrorNameFromEnum(vr::EVRSettingsError) override { return "fake error"; }
 void SetString(const char* section,const char* key,const char* value,vr::EVRSettingsError* error) override {
  CHECK(std::strcmp(section,"trackers")==0);CHECK(reads.empty());
  writes.emplace_back(key);
  *error=failed_write==key?vr::VRSettingsError_WriteFailed:vr::VRSettingsError_None;
  if(*error==vr::VRSettingsError_None)values[key]=value;
 }
 void GetString(const char* section,const char* key,char* value,uint32_t length,vr::EVRSettingsError* error) override {
  CHECK(std::strcmp(section,"trackers")==0);CHECK(writes.size()==6);
  reads.emplace_back(key);
  *error=failed_read==key?vr::VRSettingsError_ReadFailed:vr::VRSettingsError_None;
  const auto actual=mismatched_read==key?"TrackerRole_Handed":values[key];
  CHECK(actual.size()+1<=length);
  std::memcpy(value,actual.c_str(),actual.size()+1);
 }
 void SetBool(const char*,const char*,bool,vr::EVRSettingsError*) override { CHECK(false); }
 void SetInt32(const char*,const char*,int32_t,vr::EVRSettingsError*) override { CHECK(false); }
 void SetFloat(const char*,const char*,float,vr::EVRSettingsError*) override { CHECK(false); }
 bool GetBool(const char*,const char*,vr::EVRSettingsError*) override { CHECK(false);return false; }
 int32_t GetInt32(const char*,const char*,vr::EVRSettingsError*) override { CHECK(false);return 0; }
 float GetFloat(const char*,const char*,vr::EVRSettingsError*) override { CHECK(false);return 0; }
 void RemoveSection(const char*,vr::EVRSettingsError*) override { CHECK(false); }
 void RemoveKeyInSection(const char*,const char*,vr::EVRSettingsError*) override { CHECK(false); }
};

int main() {
 const std::map<std::string,std::string> expected{
  {"/devices/chadfbt/chadfbt_hip","TrackerRole_Waist"},
  {"/devices/chadfbt/chadfbt_chest","TrackerRole_Chest"},
  {"/devices/chadfbt/chadfbt_left_foot","TrackerRole_LeftFoot"},
  {"/devices/chadfbt/chadfbt_right_foot","TrackerRole_RightFoot"},
  {"/devices/chadfbt/chadfbt_left_knee","TrackerRole_LeftKnee"},
  {"/devices/chadfbt/chadfbt_right_knee","TrackerRole_RightKnee"},
 };
 const std::map<std::string,std::string> preserved{
  {"/devices/chadfbt/chadfbt_left_elbow","TrackerRole_LeftElbow"},
  {"/devices/chadfbt/chadfbt_right_elbow","TrackerRole_Handed"},
  {"/devices/another_driver/other_tracker","TrackerRole_Waist"},
 };
 FakeSettings settings;settings.values=preserved;
 // Existing wrong assignments on the six requested slots are replaced.
 for(const auto& request:expected)settings.values[request.first]="TrackerRole_Handed";
 for(int attempt=0;attempt<2;++attempt){
  settings.writes.clear();settings.reads.clear();
  const auto results=chadfbt::assign_core_tracker_roles(settings);
  CHECK(settings.writes.size()==6);CHECK(settings.reads==settings.writes);
  CHECK(settings.values.size()==expected.size()+preserved.size());
  for(const auto& request:expected)CHECK(settings.values.at(request.first)==request.second);
  for(const auto& entry:preserved)CHECK(settings.values.at(entry.first)==entry.second);
  for(std::size_t i=0;i<results.size();++i)CHECK(results[i].verified(chadfbt::core_tracker_roles[i]));
 }
 // Independent failures must not stop later writes or skip any read-back.
 for(std::size_t failed=0;failed<chadfbt::core_tracker_roles.size();++failed){
  for(int kind=0;kind<3;++kind){
   FakeSettings partial;partial.values=preserved;
   const auto key=chadfbt::core_tracker_roles[failed].key;
   if(kind==0)partial.failed_write=key;
   if(kind==1)partial.failed_read=key;
   if(kind==2)partial.mismatched_read=key;
   const auto results=chadfbt::assign_core_tracker_roles(partial);
   CHECK(partial.writes.size()==6);CHECK(partial.reads==partial.writes);
   for(std::size_t i=0;i<results.size();++i)CHECK(results[i].verified(chadfbt::core_tracker_roles[i])==(i!=failed));
   for(const auto& entry:preserved)CHECK(partial.values.at(entry.first)==entry.second);
  }
 }
 // A failed SetString is still a failure when a previous value already matches.
 settings.writes.clear();settings.reads.clear();
 settings.failed_write=chadfbt::core_tracker_roles[0].key;
 const auto results=chadfbt::assign_core_tracker_roles(settings);
 CHECK(!results[0].verified(chadfbt::core_tracker_roles[0]));
 std::cout << "Tracker role assignment regressions passed.\n";
}
