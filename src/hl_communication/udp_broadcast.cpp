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

namespace hl_communication {

UDPBroadcast::UDPBroadcast(int portRead, int portWrite)
{
    _portRead = portRead;
    _portWrite = portWrite;
    _countSend++;

    //Network initialization
    _readFd = -1;
    _writeFd = -1;
    if (_portRead != -1) {
        openRead();
    }
    if (_portWrite != -1) {
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
    _readFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (_readFd == -1) {
        std::cout << 
            "ERROR: UDPBroadcast: Unable to open read socket" << std::endl;
        std::cout << strerror(errno) << std::endl;
        return;
    }
    
    //Set broadcast permission
    int opt = 1;
    int error = setsockopt(_readFd, SOL_SOCKET, 
        SO_BROADCAST, (const char *)&opt, sizeof(opt));

    if (error != -1) {
        error = setsockopt(_readFd, SOL_SOCKET, 
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
    SOCKADDR_IN addr;
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(_portRead);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    error = bind(_readFd, (SOCKADDR*)&addr, sizeof(addr));
    if (error == -1) {
        std::cout << 
            "ERROR: UDPBroadcast: Unable to bind read socket" << std::endl;
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
    _writeFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (_writeFd == -1) {
        std::cout << 
            "ERROR: UDPBroadcast: Unable to open write socket" << std::endl;
        std::cout << strerror(errno) << std::endl;
        return;
    }

    //Set broadcast permission
    int opt = 1;
    int error = setsockopt(_writeFd, SOL_SOCKET, 
        SO_BROADCAST, (const char *)&opt, sizeof(opt));
    if (error == -1) {
        std::cout << 
            "ERROR: UDPBroadcast: Unable to configure write socket" << std::endl;
        std::cout << strerror(errno) << std::endl;
        closeWrite();
        return;
    }
}

void UDPBroadcast::closeRead()
{
    if (_readFd != -1) {
        close(_readFd);
        _readFd = -1;
    }
}
void UDPBroadcast::closeWrite()
{
    if (_writeFd != -1) {
        close(_writeFd);
        _writeFd = -1;
    }
}
        
void UDPBroadcast::broadcastMessage(const char* data, size_t len)
{
    if (_portWrite == -1) {
        return;
    }
    if (_writeFd == -1) {
        std::cout << 
            "WARNING: UDPBroadcast: closed write socket" << std::endl;
        openWrite();
        return;
    }
   
    if (_broadcastAddr.size() == 0) {
        std::cout << 
            "WARNING: UDPBroadcast: no broadcast address" << std::endl;
        retrieveBroadcastAddress();
        return;
    }
    if (_countSend > 20) {
        retrieveBroadcastAddress();
    }

    //Send message to all broadcast address
    for (size_t i=0;i<_broadcastAddr.size();i++) {
        struct sockaddr_in addr;
        bzero(&addr, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = _broadcastAddr[i];
        addr.sin_port = htons(_portWrite);
          
        int error = sendto(_writeFd, data, len, MSG_DONTWAIT,
            (struct sockaddr*)&addr, sizeof(addr));

        if (error == EAGAIN || error == EWOULDBLOCK) {
            std::cout << 
                "WARNING: UDPBroadcast: send blocked" << std::endl;
        } else if (error == -1) {
            std::cout << 
                "ERROR: UDPBroadcast: send failed" << std::endl;
            std::cout << strerror(errno) << std::endl;
        } else if (error != (int)len) {
            std::cout << 
                "ERROR: UDPBroadcast: send truncated" << std::endl;
            std::cout << strerror(errno) << std::endl;
        }
    }
    _countSend++;
}
        
bool UDPBroadcast::checkMessage(
    char* data, size_t& len,
    unsigned long* src_address, unsigned short* src_port
)
{
    if (_portRead == -1) {
        return false;
    }
    if (_readFd == -1) {
        std::cout << 
            "WARNING: UDPBroadcast: closed read socket" << std::endl;
        openRead();
        return false;
    }

    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);
    int size = recvfrom(
        _readFd, (char *) data, len, MSG_DONTWAIT,
        (struct sockaddr *) &src_addr, &addr_len
    );

    if (size == -1) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cout << 
                "ERROR: UDPBroadcast: receive failed" << std::endl;
            std::cout << strerror(errno) << std::endl;
        } 
        return false;
    } else {
        if(src_address) *src_address =(
            inet_netof( ((struct sockaddr_in*)&src_addr)->sin_addr ) << 24  
            |
            inet_lnaof( ((struct sockaddr_in*)&src_addr)->sin_addr )
        );
        if(src_port) *src_port = ntohs(((struct sockaddr_in*)&src_addr)->sin_port);
        len = size;
        return true;
    }
}
        
void UDPBroadcast::retrieveBroadcastAddress()
{
    _countSend = 0;
    _broadcastAddr.clear();
        
    struct ifaddrs* ifap;
    if (getifaddrs(&ifap) == -1) {
        std::cout << 
            "ERROR: UDPBroadcast: get broadcast address failed" << std::endl;
        std::cout << strerror(errno) << std::endl;
        return;
    }

    while (ifap != NULL) {
        SOCKADDR* addr = ifap->ifa_broadaddr;
        if (
            addr != NULL &&
            addr->sa_family == AF_INET 
        ) {
            _broadcastAddr.push_back(((SOCKADDR_IN*)addr)->sin_addr.s_addr);
        }
        ifap = ifap->ifa_next;
    }
}

}

