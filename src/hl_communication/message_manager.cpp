#include <hl_communication/message_manager.h>

#include <fstream>
#include <iostream>

namespace hl_communication
{

bool operator<(const RobotIdentifier & id1, const RobotIdentifier & id2) {
  return (id1.team_id() == id2.team_id() && id1.robot_id() < id2.robot_id()) ||
    id1.team_id() < id2.team_id();
}

MessageManager::MessageManager(const std::string & file_path) {
  loadMessages(file_path);
}

MessageManager::MessageManager(int port_read)
  : udp_receiver(new Udp_message_manager(port_read, -1))
{
}

void MessageManager::update() {
  if (udp_receiver) {
    GameMsg msg;
    while(udp_receiver->receive_message(&msg)) {
      push(msg);
    }
  }
}

void MessageManager::saveMessages(const std::string & path) {
  GameMsgCollection collection = buildGameMsgCollection();
  std::cout << "Serializing a collection of " << collection.messages_size() << " messages" << std::endl;
  std::ofstream out(path);
  if (!out.good()) {
    throw std::runtime_error("MessageManager: Failed to open file '" + path + "'");
  }
  if (!collection.SerializeToOstream(&out)) {
    throw std::runtime_error("MessageManager: Failed to write in '" + path + "'");
  }
}

double MessageManager::getStart() const {
  double min_ts = std::numeric_limits<double>::max();
  if (gc_messages.size() > 0) {
    min_ts = std::min(min_ts, gc_messages.begin()->first);
  }
  for (const auto & robot_collection : messages_by_robot) {
    if (robot_collection.second.size() > 0) {
      min_ts = std::min(min_ts, robot_collection.second.begin()->first);
    }
  }
  return min_ts;
}

MessageManager::Status MessageManager::getStatus(double time_stamp) const {
  Status status;
  for (const auto & robot_entry : messages_by_robot) {
    std::cout << "-> Checking messages from robot " << robot_entry.first.robot_id()
                << " from team " << robot_entry.first.team_id() << std::endl;
    auto it = robot_entry.second.upper_bound(time_stamp);
    // Do not include robots which have no data prior to time_stamp
    if (it == robot_entry.second.end())
      continue;
    if (it->first > time_stamp) {
      if (it == robot_entry.second.begin())
        continue;
      it--;
    }
    status.robot_messages[robot_entry.first] = it->second;
  }
  auto it = gc_messages.upper_bound(time_stamp);
  if (it == gc_messages.end())
    return status;
  if (it->first > time_stamp) {
    if (it == gc_messages.begin())
      return status;
    it--;
  }
  status.gc_message = it->second;
  return status;
}

void MessageManager::push(const RobotMsg & msg) {
  if (!msg.has_robot_id()) {
    throw std::runtime_error("MessageManager can only handle identified RobotMsg");
  }
  if (!msg.has_time_stamp()) {
    throw std::runtime_error("MessageManager can only handle time_stamped RobotMsg");
  }
  messages_by_robot[msg.robot_id()][msg.time_stamp()] = msg;
}

void MessageManager::push(const GCMsg & msg) {
  if (!msg.has_time_stamp()) {
    throw std::runtime_error("MessageManager can only handle time_stamped GCMsg");
  }
  gc_messages[msg.time_stamp()] = msg;
}

void MessageManager::push(const GameMsg & msg) {
  if (msg.has_robot_msg()) {
    push(msg.robot_msg());
    
  } else if (msg.has_gc_msg()) {
    push(msg.gc_msg());
  } else {
    throw std::runtime_error("Failed to read GameMsg, not a RobotMsg neither a GCMsg");
  }
}

void MessageManager::push(const GameMsgCollection & collection) {
  for (const GameMsg & msg : collection.messages()) {
    push(msg);
  }
}

void MessageManager::loadMessages(const std::string & file_path) {
  std::ifstream in(file_path, std::ios::binary);
  if (!in.good()) {
    throw std::runtime_error("Failed to open file at '" + file_path + "'");
  }
  GameMsgCollection messages_collection;
  if (!messages_collection.ParseFromIstream(&in)) {
    throw std::runtime_error("Failed to parse messages in file '" + file_path + "'");
  }
  std::cout << "Pushing " << messages_collection.messages_size() << " messages in MM" << std::endl;
  for (const GameMsg msg : messages_collection.messages()) {
    push(msg);
  }
}

GameMsgCollection MessageManager::buildGameMsgCollection() const {
  GameMsgCollection result;
  for (const auto & gc_entry : gc_messages) {
    result.add_messages()->mutable_gc_msg()->CopyFrom(gc_entry.second);
  }
  for (const auto & robot_entry : messages_by_robot) {
    for (const auto & robot_msg_entry : robot_entry.second) {
      result.add_messages()->mutable_robot_msg()->CopyFrom(robot_msg_entry.second);
    }
  }
  return result;
}

}
