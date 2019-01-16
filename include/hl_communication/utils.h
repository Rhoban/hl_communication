#pragma once

#include <string>

namespace hl_communication
{

#define HL_DEBUG                                   \
  (std::string(__FUNCTION__) + ":"                 \
   + hl_communication::getBaseName(__FILE__) + ":" \
   + std::to_string(__LINE__)  + ": ")

/**
 * Return the name of the file at the given path:
 * e.g getBaseName("toto/file.cpp") returns "file.cpp"
 */
std::string getBaseName(const std::string & path);

/**
 * Return time_since_epoch in a integer value (unit: microseconds)
 */
uint64_t getTimeStamp();

}
