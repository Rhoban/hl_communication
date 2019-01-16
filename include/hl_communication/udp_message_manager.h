#ifndef HL_COMMUNICATION_UDPLOADCONFIG_H
#define  HL_COMMUNICATION_UDPLOADCONFIG_H

#include <hl_communication/wrapper.pb.h>
#include <google/protobuf/text_format.h>
#include <queue>
#include <hl_communication/udp_broadcast.h>
#include <mutex>
#include <thread>

namespace hl_communication {

class Udp_message_manager {
    private:
    unsigned long int _packet_no; 

    int _portRead;
    int _portWrite;

    bool _continue_to_run;
    std::thread * _thread;
    std::mutex _mutex;

    std::queue<hl_communication::GameMsg> _messages;

    hl_communication::UDPBroadcast* _broadcaster;

    void _run();

    public:
    bool receive_message( hl_communication::GameMsg* message );

    /**
     * Send a message, Packet number is not filled automatically.
     * You have to fill it before sending the message.
     */
    void send_message( const hl_communication::GameMsg & message );

    /**
     * Fill automatically the Packet Number and send the message.
     */
    void send_message( hl_communication::GameMsg * message );

    Udp_message_manager(int portRead, int portWrite);
    ~Udp_message_manager();
};

}

#endif
