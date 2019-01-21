#include <hl_communication/utils.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std::chrono;

namespace hl_communication
{

void readFromFile(const std::string & path, google::protobuf::Message * msg) {
  msg->Clear();
  std::ifstream in(path);
  if (!in.good()) {
    throw std::runtime_error(HL_DEBUG + " failed to open file '" + path + "'");
  }
  msg->ParseFromIstream(&in);
}

void writeToFile(const std::string & path, const google::protobuf::Message & msg) {
  std::ofstream out(path, std::ios::binary);
  if (!out.good()) {
    throw std::runtime_error(HL_DEBUG + " failed to open file '" + path + "'");
  }
  msg.SerializeToOstream(&out);
}

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

void invertPosition(PositionDistribution * position) {
  position->set_x(-position->x());
  position->set_y(-position->y());
}

void invertAngle(AngleDistribution * angle) {
  double alpha = angle->mean() + M_PI;
  if (alpha > M_PI) {
    alpha -= 2 * M_PI;
  }
  angle->set_mean(alpha);
}

void invertPose(PoseDistribution * pose) {
  if (pose->has_position()) {
    invertPosition(pose->mutable_position());
  }
  if (pose->has_dir()) {
    invertAngle(pose->mutable_dir());
  }
}

}
