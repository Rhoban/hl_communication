#pragma once

#include <hl_communication/udp_message_manager.h>
#include <hl_communication/wrapper.pb.h>

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

  class Status {
  public:
    std::map<RobotIdentifier, RobotMsg> robot_messages;
    GCMsg gc_message;
  };
  
  /**
   * Load multiple messages from an existing Protobuf file
   */
  MessageManager(const std::string & file_path);

  /**
   * Open a message manager which will listen to messages on the given port
   */
  MessageManager(int port_read);

  void update();

  void saveMessages(const std::string & path);

  double getStart() const;

  /**
   * Build a global game status with the last message of each entity prior to
   * the given time_stamp
   */
  Status getStatus(double time_stamp) const;

private:

  void push(const RobotMsg & msg);
  void push(const GCMsg & msg);
  void push(const GameMsg & msg);
  void push(const GameMsgCollection & collection);

  void loadMessages(const std::string & file_path);

  /**
   * Gather all the received messages in a GameMsgCollection
   */
  GameMsgCollection buildGameMsgCollection() const;
  
  /**
   * Messages received ordered by robot identifier
   */
  std::map<RobotIdentifier, TimedRobotMsgCollection> messages_by_robot;

  /**
   * Game Controller messages received ordered by time_stamp
   */
  std::map<double, GCMsg> gc_messages;

  std::unique_ptr<Udp_message_manager> udp_receiver;
};


}
