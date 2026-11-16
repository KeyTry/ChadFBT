#pragma once
#include "openvr.h"
#include <array>
#include <cstring>

namespace chadfbt {
struct TrackerRoleAssignment {
 const char* key;
 const char* role;
};

// Explicitly requested body outputs only. Serial names are driver slots, while
// these values are SteamVR roles. Elbows and all other drivers remain untouched.
inline constexpr std::array<TrackerRoleAssignment,6> core_tracker_roles{{
 {"/devices/chadfbt/chadfbt_hip", "TrackerRole_Waist"},
 {"/devices/chadfbt/chadfbt_chest", "TrackerRole_Chest"},
 {"/devices/chadfbt/chadfbt_left_foot", "TrackerRole_LeftFoot"},
 {"/devices/chadfbt/chadfbt_right_foot", "TrackerRole_RightFoot"},
 {"/devices/chadfbt/chadfbt_left_knee", "TrackerRole_LeftKnee"},
 {"/devices/chadfbt/chadfbt_right_knee", "TrackerRole_RightKnee"},
}};

struct TrackerRoleAssignmentResult {
 vr::EVRSettingsError write_error=vr::VRSettingsError_None;
 vr::EVRSettingsError read_error=vr::VRSettingsError_None;
 char read_value[128]{};

 bool verified(const TrackerRoleAssignment& requested) const {
  return write_error==vr::VRSettingsError_None
      && read_error==vr::VRSettingsError_None
      && std::strcmp(read_value,requested.role)==0;
 }
};

// IVRSettings writes are independent: a failure can leave earlier writes in
// place. Attempt and read back all six so callers report the complete outcome.
inline std::array<TrackerRoleAssignmentResult,6> assign_core_tracker_roles(vr::IVRSettings& settings) {
 std::array<TrackerRoleAssignmentResult,6> results{};
 for(std::size_t i=0;i<core_tracker_roles.size();++i){
  const auto& request=core_tracker_roles[i];
  settings.SetString(vr::k_pch_Trackers_Section,request.key,request.role,&results[i].write_error);
 }
 for(std::size_t i=0;i<core_tracker_roles.size();++i){
  auto& result=results[i];
  settings.GetString(vr::k_pch_Trackers_Section,core_tracker_roles[i].key,
      result.read_value,sizeof(result.read_value),&result.read_error);
  result.read_value[sizeof(result.read_value)-1]='\0';
 }
 return results;
}
} // namespace chadfbt
