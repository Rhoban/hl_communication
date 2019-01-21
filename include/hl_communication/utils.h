#pragma once

#include <hl_communication/position.pb.h>

#include <google/protobuf/message.h>

#include <string>

namespace hl_communication
{

#define HL_DEBUG                                   \
  (std::string(__FUNCTION__) + ":"                 \
   + hl_communication::getBaseName(__FILE__) + ":" \
   + std::to_string(__LINE__)  + ": ")

void readFromFile(const std::string & path, google::protobuf::Message * msg);

void writeToFile(const std::string & path, const google::protobuf::Message & msg);

/**
 * Return the name of the file at the given path:
 * e.g getBaseName("toto/file.cpp") returns "file.cpp"
 */
std::string getBaseName(const std::string & path);

/**
 * Return time_since_epoch in a integer value (unit: microseconds)
 */
uint64_t getTimeStamp();

/**
 * Convert a human readable string to a 8 bytes ip address
 */
uint64_t stringToIP(const std::string & str);

/**
 * Convert a 8 bytes ip address to a human readable string
 */
std::string ipToString(uint64_t ip_address);

/**
 * Invert the side of the angle message (x-axis toward left or right of team area)
 */
void invertPosition(PositionDistribution * position);

/**
 * Invert the side of the angle message (x-axis toward left or right of team area)
 */
void invertAngle(AngleDistribution * position);

/**
 * Invert the side of the provided pose message (x-axis toward left or right of team area)
 */
void invertPose(PoseDistribution * pose);

}
