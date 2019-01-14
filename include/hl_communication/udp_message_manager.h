#ifndef HL_COMMUNICATION_UDPLOADCONFIG_H
#define  HL_COMMUNICATION_UDPLOADCONFIG_H

#include <hl_communication/wrapper.pb.h>
#include <google/protobuf/text_format.h>
#include <queue>
#include <hl_communication/udp_broadcast.h>
#include <chrono>
#include <mutex>
#include <thread>

namespace hl_communication {

class UDPLoadConfig {
    private:
    int _portRead;
    int _portWrite;

    bool _continue_to_run;
    std::thread * _thread;
    std::mutex _mutex;

    std::queue<hl_communication::GameMsg> _messages;

    hl_communication::UDPBroadcast* _broadcaster;
    std::chrono::time_point<std::chrono::system_clock> _start;

    void _run();

    public:
    bool receive_message( hl_communication::GameMsg* message );
    void send_message( const hl_communication::GameMsg & message );

    UDPLoadConfig(int portRead, int portWrite);
    ~UDPLoadConfig();
};

}

#endif
