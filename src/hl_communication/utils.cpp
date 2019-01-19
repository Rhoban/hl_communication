#include <hl_communication/utils.h>

#include <chrono>
#include <sstream>


#include <iostream>

using namespace std::chrono;

namespace hl_communication
{

uint64_t getTimeStamp() {
  return duration_cast<duration<uint64_t,std::micro>>(steady_clock::now().time_since_epoch()).count();
}

std::string getBaseName(const std::string & path) {
  size_t idx = path.find_last_of('/');
  if (idx == std::string::npos) {
    return path;
  }
  return path.substr(idx+1);
}

uint64_t stringToIP(const std::string & str) {
  std::stringstream ss(str);
  uint64_t result = 0;
  std::string element;
  while(getline(ss, element, '.')) {
    uint64_t elem_value = std::stoi(element);
    result = (result << 8) + elem_value;
  }
  return result;
}

std::string ipToString(uint64_t ip) {
  std::ostringstream oss;
  oss << (ip>>24 &0xFF)  << "."
      << (ip>>16 &0xFF)  << "."
      << (ip>>8  &0xFF)  << "."
      << (ip     &0xFF);
  return oss.str();
}

}
