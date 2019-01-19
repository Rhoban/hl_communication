#pragma once

#include <vector>

#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>

namespace hl_communication {

/**
 * UDPBroadcast
 *
 * Simple UDP non blocking 
 * broadcaster
 */
class UDPBroadcast
{
public:

  /**
   * Initializing with listening port
   */
  UDPBroadcast(int port_read, int port_write);

  /**
   * Close opened connections
   */
  ~UDPBroadcast();
        
  /**
   * Start or restart the listening and ouput
   * connection
   */
  void openRead();
  void openWrite();

  /**
   * Close listening and output connection
   */
  void closeRead();
  void closeWrite();

  /**
   * Broadcast given UDP message
   */
  void broadcastMessage(const char* data, size_t len);

  /**
   * Return true of the given bufffer has been 
   * filled with incomming data. Len should contain the size of the buffer.
   * At the end of the function len is updated and contain the size of
   * the received message.
   */
  bool checkMessage(char* data, size_t* len,
                    uint64_t * src_address=NULL, uint32_t * src_port=NULL);

private:

  /**
   * Listening and writting port.
   * If port_write is -1, packet sending 
   * is disabled
   */
  int port_read;
  int port_write;
        
  /**
   * Incoming and outcoming connection
   */
  int read_fd;
  int write_fd;

  /**
   * Count the number of send packets since
   * last interface listing
   */
  int count_send;

  /**
   * Broadcast address list
   */
  std::vector<int> broadcast_addr;

  /**
   * Retrieve for all interfaces the broadcast address
   */
  void retrieveBroadcastAddress();
};

}
