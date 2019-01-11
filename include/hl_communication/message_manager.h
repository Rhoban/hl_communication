#pragma once

#include "hl_communication/wrapper.pb.h"

namespace hl_communication
{

/**
 * RobotIdentifier are ordered by team and then by robot_id
 */ 
bool operator<(const RobotIdentifier & id1, const RobotIdentifier & id2);

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
  void push(const GCMsg & msg);
  void push(const GameMsg & msg);
  void push(const GameMsgCollection & collection);

  void loadMessages(const std::string & file_path);
  
  /**
   * Messages received ordered by robot identifier
   */
  std::map<RobotIdentifier, TimedRobotMsgCollection> messages_by_robot;

  /**
   * Game Controller messages received ordered by time_stamp
   */
  std::map<double, GCMsg> game_controller_messages;

  //TODO: add UDP listener
};


}
