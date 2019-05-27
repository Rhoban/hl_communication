#pragma once

#include <hl_communication/udp_message_manager.h>
#include <hl_communication/wrapper.pb.h>

namespace hl_communication
{
/**
 * RobotIdentifier are ordered by team and then by robot_id
 */
bool operator<(const RobotIdentifier& id1, const RobotIdentifier& id2);

/**
 * Order first by ip, then by port and finally by packet_no
 *
 * Throws an error if src_ip or src_port of one of the message is not set
 */
bool operator<(const MsgIdentifier& id1, const MsgIdentifier& id2);

/**
 * Manages an history of message based either on logs or on a server
 */
class MessageManager
{
public:
  typedef std::map<uint64_t, RobotMsg> TimedRobotMsgCollection;

  /**
   * Uniquely identify a source of packets
   */
  class SourceIdentifier
  {
  public:
    uint64_t src_ip;
    uint32_t src_port;
  };

  class Status
  {
  public:
    std::map<RobotIdentifier, RobotMsg> robot_messages;
    GCMsg gc_message;

    /**
     * Get the robot messages ordered by team
     */
    std::map<uint32_t, std::vector<RobotMsg>> getRobotsByTeam() const;
  };

  /**
   * Default implementation: no receivers opened
   */
  MessageManager();

  /**
   * Load multiple messages from an existing Protobuf file
   */
  MessageManager(const std::string& file_path);

  /**
   * Open a message manager which will listen to messages on the given port
   */
  MessageManager(int port_read, bool auto_discover_ports);

  /**
   * Specifies multiple ports for reading entries
   */
  MessageManager(const std::vector<int>& ports);

  void update();

  void saveMessages(const std::string& path);

  /**
   * Return the timestamp of the first message received
   */
  uint64_t getStart() const;
  /**
   * Return the timestamp of the last message received
   */
  uint64_t getEnd() const;

  /**
   * Build a global game status with the last message of each entity prior to
   * the given time_stamp
   *
   * - The system_clock option allows to specify that the time_stamp is not
   *    based on steady clock, but on a system clock
   */
  Status getStatus(uint64_t time_stamp, bool system_clock = false) const;

  /**
   * Ignore messages older than "time_stamp - history_length"
   */
  Status getStatus(uint64_t time_stamp, uint64_t history_length, bool system_clock = false) const;

  /**
   * Set the offset in us between steady_clock and system_clock (time_since_epoch)
   */
  void setOffset(int64_t offset);

  /**
   * Get the offset in us between steady_clock and system_clock (time_since_epoch)
   */
  int64_t getOffset() const;

private:
  /**
   * Open an udp receiver on given port, throws logic_error if port is already opened with this message manager
   */
  void openReceiver(int port);

  /**
   * Message should be stored in receivedmessages
   */
  void push(const RobotMsg& msg);

  /**
   * Message should be stored in receivedmessages
   */
  void push(const GCMsg& msg);
  void push(const GameMsg& msg);
  void push(const GameMsgCollection& collection);

  void loadMessages(const std::string& file_path);

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
  std::map<uint64_t, GCMsg> gc_messages;

  /**
   * Offset between clock used for internal time_stamps and UTC time_stamp [us]:
   * msg.time_stamp + time_offset = utc_time_stamp
   */
  int64_t clock_offset;

  /**
   * Stock all the received messages, making sure they are unique
   */
  std::map<MsgIdentifier, GameMsg> received_messages;

  std::map<int, std::unique_ptr<UDPMessageManager>> udp_receivers;

  /**
   * Stores the list of providers of game controller messages.
   */
  std::set<SourceIdentifier> gc_sources;

  /**
   * When activated, uses automatically game controller messages to open udp receivers for the default ports, and closes
   * other ports
   */
  bool auto_discover_ports;
};

bool operator<(const MessageManager::SourceIdentifier& id1, const MessageManager::SourceIdentifier& id2);

}  // namespace hl_communication
