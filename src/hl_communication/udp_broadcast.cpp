#include <iostream>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <linux/if.h>
#include <errno.h>
#include <string.h>

#include <hl_communication/udp_broadcast.h>
#include <hl_communication/utils.h>

namespace hl_communication {

UDPBroadcast::UDPBroadcast(int port_read_, int port_write_)
  : port_read(port_read_), port_write(port_write_), count_send(0)
{
  //Network initialization
  read_fd = -1;
  write_fd = -1;
  if (port_read != -1) {
    openRead();
  }
  if (port_write != -1) {
    openWrite();
  }
  retrieveBroadcastAddress();
}

UDPBroadcast::~UDPBroadcast()
{
  closeWrite();
  closeRead();
}

void UDPBroadcast::openRead()
{
  //Close current connection if open
  closeRead();

  //Open read socket
  read_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (read_fd == -1) {
    std::cout << 
      "ERROR: UDPBroadcast: Unable to open read socket" << std::endl;
    std::cout << strerror(errno) << std::endl;
    return;
  }
    
  //Set broadcast permission
  int opt = 1;
  int error = setsockopt(read_fd, SOL_SOCKET, 
                         SO_BROADCAST, (const char *)&opt, sizeof(opt));

  if (error != -1) {
    error = setsockopt(read_fd, SOL_SOCKET, 
                       SO_REUSEPORT, (const char *)&opt, sizeof(opt));
  }

  if (error == -1) {
    std::cout << 
      "ERROR: UDPBroadcast: Unable to configure read socket" << std::endl;
    std::cout << strerror(errno) << std::endl;
    closeRead();
    return;
  }

  //Bind socket to listening port
  struct sockaddr_in addr;
  bzero(&addr, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_read);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  error = bind(read_fd, (struct sockaddr*)&addr, sizeof(addr));
  if (error == -1) {
    std::cout << "ERROR: UDPBroadcast: Unable to bind read socket" << std::endl;
    std::cout << strerror(errno) << std::endl;
    closeRead();
    return;
  }
}
void UDPBroadcast::openWrite()
{
  //Close current connection if open
  closeWrite();

  //Open read socket
  write_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (write_fd == -1) {
    std::cout << "ERROR: UDPBroadcast: Unable to open write socket" << std::endl;
    std::cout << strerror(errno) << std::endl;
    return;
  }

  //Set broadcast permission
  int opt = 1;
  int error = setsockopt(write_fd, SOL_SOCKET, 
                         SO_BROADCAST, (const char *)&opt, sizeof(opt));
  if (error == -1) {
    std::cout << "ERROR: UDPBroadcast: Unable to configure write socket" << std::endl;
    std::cout << strerror(errno) << std::endl;
    closeWrite();
    return;
  }
}

void UDPBroadcast::closeRead()
{
  if (read_fd != -1) {
    close(read_fd);
    read_fd = -1;
  }
}
void UDPBroadcast::closeWrite()
{
  if (write_fd != -1) {
    close(write_fd);
    write_fd = -1;
  }
}
        
void UDPBroadcast::broadcastMessage(const char* data, size_t len)
{
  if (port_write == -1) {
    return;
  }
  if (write_fd == -1) {
    std::cout << "WARNING: UDPBroadcast: closed write socket" << std::endl;
    openWrite();
    return;
  }
   
  if (broadcast_addr.size() == 0) {
    std::cout << "WARNING: UDPBroadcast: no broadcast address" << std::endl;
    retrieveBroadcastAddress();
    return;
  }
  if (count_send > 20) {
    retrieveBroadcastAddress();
  }

  //Send message to all broadcast address
  for (size_t i=0;i<broadcast_addr.size();i++) {
    struct sockaddr_in addr;
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = broadcast_addr[i];
    addr.sin_port = htons(port_write);
          
    int error = sendto(write_fd, data, len, MSG_DONTWAIT,
                       (struct sockaddr*)&addr, sizeof(addr));

    if (error == EAGAIN || error == EWOULDBLOCK) {
      std::cout << "WARNING: UDPBroadcast: send blocked" << std::endl;
    } else if (error == -1) {
      std::cout << "ERROR: UDPBroadcast: send failed" << std::endl;
      std::cout << strerror(errno) << std::endl;
    } else if (error != (int)len) {
      std::cout << "ERROR: UDPBroadcast: send truncated" << std::endl;
      std::cout << strerror(errno) << std::endl;
    }
  }
  count_send++;
}
        
bool UDPBroadcast::checkMessage(
  char* data, size_t* len,
  uint64_t * src_address, uint32_t * src_port
  )
{
  if (port_read == -1) {
    return false;
  }
  if (read_fd == -1) {
    std::cout << "WARNING: UDPBroadcast: closed read socket" << std::endl;
    openRead();
    return false;
  }

  struct sockaddr_in src_addr;
  socklen_t addr_len = sizeof(src_addr);
  int size = recvfrom(read_fd, (char *) data, *len, MSG_DONTWAIT,
                      (struct sockaddr *) &src_addr, &addr_len);

  if (size == -1) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      std::cout << "ERROR: UDPBroadcast: receive failed" << std::endl;
      std::cout << strerror(errno) << std::endl;
    }
    return false;
  } else {
    if(src_address) {
      char addr_str[16];// XXX.XXX.XXX.XXX + \0
      socklen_t addr_len = 16;
      if (inet_ntop(AF_INET, &(src_addr.sin_addr), addr_str, addr_len) == nullptr) {
        std::cout << "ERROR: UDPBroadcast: receive failed" << std::endl;
        std::cout << strerror(errno) << std::endl;
      }
      *src_address = stringToIP(std::string(addr_str,addr_len));
    }
    if(src_port) {
      *src_port = ntohs(src_addr.sin_port);
    }
    *len = size;
    return true;
  }
}
        
void UDPBroadcast::retrieveBroadcastAddress()
{
  count_send = 0;
  broadcast_addr.clear();
        
  struct ifaddrs* ifap;
  if (getifaddrs(&ifap) == -1) {
    std::cout << "ERROR: UDPBroadcast: get broadcast address failed" << std::endl;
    std::cout << strerror(errno) << std::endl;
    return;
  }

  while (ifap != NULL) {
    struct sockaddr * addr = ifap->ifa_broadaddr;
    if (addr != NULL && addr->sa_family == AF_INET) {
      broadcast_addr.push_back(((struct sockaddr_in *)addr)->sin_addr.s_addr);
    }
    ifap = ifap->ifa_next;
  }
}

}

