#pragma once

#include <hl_communication/udp_message_manager.h>
#include <hl_communication/wrapper.pb.h>

namespace hl_communication
{
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

  enum TeamColor
  {
    RED,
    BLUE,
    UNKNOWN,
    CONFLICT
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

  TeamColor getTeamColor(uint64_t utc_ts, const RobotIdentifier& robot_id) const;

  const std::map<RobotIdentifier, TeamColor>& getRobotsColors() const;

  /**
   * Set the offset in us between steady_clock and system_clock (time_since_epoch)
   */
  void setOffset(int64_t offset);

  /**
   * Get the offset in us between steady_clock and system_clock (time_since_epoch)
   */
  int64_t getOffset() const;

  void loadMessages(const std::string& file_path);

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
   * Check if we already
   */

  bool hasMainGCSource();

  /**
   * Message should be stored in receivedmessages
   */
  void push(const GCMsg& msg, bool isWantedMessage);
  void push(const GameMsg& msg);
  void push(const GameMsgCollection& collection);

  /**
   * Gather all the received messages in a GameMsgCollection
   */
  GameMsgCollection buildGameMsgCollection() const;

  /**
   * Messages received ordered by robot identifier and then by emission utc_time_stamp
   */
  std::map<RobotIdentifier, TimedRobotMsgCollection> messages_by_robot;

  /**
   * Game Controller messages received ordered by emission time_stamp utc
   */
  std::map<uint64_t, GCMsg> main_gc_messages;

  /**
   * Unwanted Game Controller messages received ordered by emission time_stamp utc
   */
  std::map<uint64_t, GCMsg> interfering_gc_messages;

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
   * Stores the main provider of game controller messages.
   */
  SourceIdentifier main_gc_source;

  /**
   * Stores the list of unwanted provider of game controller messages.
   */
  std::set<SourceIdentifier> interfering_gc_sources;

  /**
   * When activated, uses automatically game controller messages to open udp receivers for the default ports, and closes
   * other ports
   */
  bool auto_discover_ports;

  std::map<RobotIdentifier, TeamColor> active_robots_colors;
};

bool operator<(const MessageManager::SourceIdentifier& id1, const MessageManager::SourceIdentifier& id2);

bool operator!=(const MessageManager::SourceIdentifier& id1, const MessageManager::SourceIdentifier& id2);

}  // namespace hl_communication
