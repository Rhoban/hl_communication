#pragma once

#include "hl_communication/wrapper.pb.h"

namespace hl_communication
{

/**
 * Manages an history of message based either on logs or on a server
 */
class MessageManager {
public:
  typedef std::map<double, RobotMsg> TimedRobotMsgCollection;
  
  /**
   * Load multiple messages from an existing Protobuf file
   */
  MessageManager(const std::string & file_path);

  /**
   * Open a message manager which will listen to messages on the given port
   */
  MessageManager(int port);
  
private:

  void push(const RobotMsg & msg);
  void push(const RobotMsgCollection & collection);
  void push(const GameControllerMsg & msg);
  void push(const GameControllerMsgCollection & collection);

  void loadMessages(const std::string & file_path);
  
  /**
   * Messages received ordered by robot identifier
   */
  std::map<RobotIdentifier, TimedRobotMsgCollection> messages_by_robot;

  /**
   * Messages received ordered 
   */
  GameControllerMsgCollection game_controller_messages;

  //TODO: add UDP listener
};


}
