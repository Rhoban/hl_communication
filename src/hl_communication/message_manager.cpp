#include "hl_communication/message_manager.h"

#include <fstream>

namespace hl_communication
{

bool operator<(const RobotIdentifier & id1, const RobotIdentifier & id2) {
  return (id1.team_id() == id2.team_id() && id1.robot_id() < id2.robot_id()) ||
    id1.team_id() < id2.team_id();
}

MessageManager::MessageManager(const std::string & file_path) {
  loadMessages(file_path);
}

MessageManager::MessageManager(int port) {
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
  game_controller_messages[msg.time_stamp()] = msg;
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
  for (const GameMsg msg : messages_collection.messages()) {
    push(msg);
  }
}

}
