#pragma once

#include <hl_communication/game_controller.pb.h>
#include <hl_communication/position.pb.h>

#include <google/protobuf/message.h>

#include <string>

namespace hl_communication
{
#define HL_DEBUG                                                                                                       \
  (std::string(__FUNCTION__) + ":" + hl_communication::getBaseName(__FILE__) + ":" + std::to_string(__LINE__) + ": ")

void readFromFile(const std::string& path, google::protobuf::Message* msg);

void writeToFile(const std::string& path, const google::protobuf::Message& msg);

/**
 * Return the name of the file at the given path:
 * e.g getBaseName("toto/file.cpp") returns "file.cpp"
 */
std::string getBaseName(const std::string& path);

/**
 * Return time_since_epoch in a integer value (unit: microseconds)
 */
uint64_t getTimeStamp();

/**
 * Return the offset from steady_clock to system_clock in us:
 * steady_clock + offset = system_clock
 */
int64_t getSteadyClockOffset();

/**
 * Convert a human readable string to a 8 bytes ip address
 */
uint64_t stringToIP(const std::string& str);

/**
 * Convert a 8 bytes ip address to a human readable string
 */
std::string ipToString(uint64_t ip_address);

/**
 * Invert the side of the angle message (x-axis toward left or right of team area)
 */
void invertPosition(PositionDistribution* position);

/**
 * Invert the side of the angle message (x-axis toward left or right of team area)
 */
void invertAngle(AngleDistribution* position);

/**
 * Invert the side of the provided pose message (x-axis toward left or right of team area)
 */
void invertPose(PoseDistribution* pose);

/**
 * Return false if player is not specifically penalized in GCMsg. This means
 * that even if GCMsg does not concern 'team_id', the answer will be false.
 * robot_id starts from 1
 */
bool isPenalized(const GCMsg& msg, int team_id, int robot_id);

}  // namespace hl_communication
