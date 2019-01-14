#include <hl_communication/udp_message_manager.h>
#include <iostream>

#define PACKET_MAX_SIZE 1000

namespace hl_communication {

UDPLoadConfig::UDPLoadConfig(int portRead, int portWrite){
    _portRead = portRead;
    _portWrite = portWrite;
    _broadcaster = new hl_communication::UDPBroadcast(portRead, portWrite);
    _start = std::chrono::system_clock::now();
    if(portRead != -1 ){
        _broadcaster->openRead();
        _thread = new std::thread( [this](){this->_run();} ); 
    }
    if(portWrite != -1){
        _broadcaster->openWrite();
    }
}

void UDPLoadConfig::_run(){
    //Receiving informations
    unsigned char data[PACKET_MAX_SIZE+1];
    data[PACKET_MAX_SIZE] = '\0';
    size_t len = PACKET_MAX_SIZE; // TODO !
    while (_broadcaster->checkMessage(data, len) and _continue_to_run) {
        if (len == PACKET_MAX_SIZE) {
            std::cout << "Packet are too long !" << std::endl;
            continue;
        }
        std::string string_data((char*) data);
        hl_communication::GameMsg game_msg;
        if( ! game_msg.ParseFromString( string_data ) ){
            std::cerr << "Invalid format for the packet" << std::endl;
        }

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end - _start;
        
        //Assign reception timestamp
        if( game_msg.has_gc_msg() ){
            game_msg.mutable_gc_msg()->set_time_stamp( diff.count() );
        }
        if( game_msg.has_robot_msg() ){
            game_msg.mutable_robot_msg()->set_time_stamp(  diff.count() );
        }
        _mutex.lock();
        _messages.push( game_msg );
        _mutex.unlock();
    }
}

UDPLoadConfig::~UDPLoadConfig(){
    _continue_to_run = false;
    _thread->join();
    delete _thread;
    _broadcaster->closeRead();
    _broadcaster->closeWrite();
    delete _broadcaster;
}

 
bool UDPLoadConfig::receive_message( hl_communication::GameMsg * message ){
    bool res = false;
    if(_portRead != -1 ){
        _mutex.lock();
        if( !_messages.empty() ){
            *message = _messages.front();
            _messages.pop();
            res = true;
        }
        _mutex.unlock();
    }
    return res;
}

void UDPLoadConfig::send_message(
    const hl_communication::GameMsg & message
){
    std::string raw_message;
    if( ! message.SerializeToString(&raw_message) ){
        std::cerr << "Invalid Message !" << std::endl;
        return;
    }
    _broadcaster->broadcastMessage(
        (unsigned char*) raw_message.c_str(), raw_message.size()
    );
}

}

