#pragma once

#include <hl_communication/wrapper.pb.h>
#include <google/protobuf/text_format.h>
#include <queue>
#include <hl_communication/udp_broadcast.h>
#include <mutex>
#include <thread>

namespace hl_communication {

/**
 * Return the default team_port
 */
int getDefaultTeamPort(int team_id);

class UDPMessageManager {
private:
  /**
   * Counts the number of messages sent and 
   */
  uint64_t packet_sent_no; 

  /**
   * Counts the number of messages received by the game controller which is not
   * using protobuf. Required because the GC is not currently sending packet_no
   */
  uint64_t packet_gc_no;

  int port_read;
  int port_write;

  bool continue_to_run;
  std::unique_ptr<std::thread> thread;
  std::mutex mutex;

  std::queue<hl_communication::GameMsg> messages;

  std::unique_ptr<hl_communication::UDPBroadcast> broadcaster;

  void run();

public:
  bool receiveMessage(hl_communication::GameMsg* message);

  /**
   * Send a message, Packet number is not filled automatically.
   * You have to fill it before sending the message.
   */
  void sendMessage(const hl_communication::GameMsg & message);

  /**
   * Fill automatically the Packet Number and send the message.
   */
  void sendMessage(hl_communication::GameMsg * message);

  UDPMessageManager(int port_read, int port_write);
  ~UDPMessageManager();
};

}
