#include "hl_communication/message_manager.h"

namespace hl_communication
{

MessageManager::MessageManager(const std::string & file_path) {
}

MessageManager::MessageManager(int port) {
}

void MessageManager::push(const RobotMsg & msg) {
  if (!msg.has_robot_id()) {
    throw std::runtime_error("MessageManager can only handle identified messages");
  }
  if (!msg.has_time_stamp()) {
    throw std::runtime_error("MessageManager can only handle time_stamped messages");
  }
  messagesByRobot[msg.robot_id()][msg.time_stamp()] = msg;
}

void MessageManager::push(const RobotMsgCollection & collection) {
  for (const RobotMsg & msg : collection.messages()) {
    push(msg);
  }
}

void MessageManager::push(const GameControllerMsg & msg) {
}

void MessageManager::push(const GameControllerMsgCollection & msg) {
}

void MessageManager::loadMessages(const std::string & file_path) {
}

}
