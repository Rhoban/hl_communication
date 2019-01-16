#include <hl_communication/udp_message_manager.h>

#include <chrono>
#include <iostream>

#define PACKET_MAX_SIZE 10000

using namespace std::chrono;

namespace hl_communication {

Udp_message_manager::Udp_message_manager(int portRead, int portWrite){
    _continue_to_run = true;
    _portRead = portRead;
    _portWrite = portWrite;
    _broadcaster = new hl_communication::UDPBroadcast(portRead, portWrite);
    if(portRead != -1 ){
        _broadcaster->openRead();
        _thread = new std::thread( [this](){this->_run();} ); 
    }
    if(portWrite != -1){
        _broadcaster->openWrite();
    }
}

void Udp_message_manager::_run(){
    //Receiving informations
    unsigned char data[PACKET_MAX_SIZE];
    size_t len = PACKET_MAX_SIZE; // TODO !
    while ( _continue_to_run) {
        if( ! _broadcaster->checkMessage(data, len) ) continue;
        if (len == PACKET_MAX_SIZE) {
            std::cout << "Packet are too long !" << std::endl;
            continue;
        }
        std::string string_data((char*) data, len);
        hl_communication::GameMsg game_msg;
        if( ! game_msg.ParseFromString( string_data ) ){
            std::cerr << "Invalid format for a packet of size: " << len << std::endl;
            continue;
        }

        double time_stamp =
          duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
        
        //Assign reception timestamp
        if( game_msg.has_gc_msg() ){
            game_msg.mutable_gc_msg()->set_time_stamp(time_stamp);
        }
        if( game_msg.has_robot_msg() ){
            game_msg.mutable_robot_msg()->set_time_stamp(time_stamp);
        }
        _mutex.lock();
        _messages.push( game_msg );
        _mutex.unlock();
    }
}

Udp_message_manager::~Udp_message_manager(){
    _continue_to_run = false;
    if(_portRead != -1){
        _broadcaster->closeRead();
        _thread->join();
        delete _thread;
    }
    if(_portWrite != -1){
        _broadcaster->closeWrite();
    }
    delete _broadcaster;
}

 
bool Udp_message_manager::receive_message( hl_communication::GameMsg * message ){
    bool res = false;
    if(_portRead != -1 ){
        _mutex.lock();
        if( !_messages.empty() ){
            message->Clear();
            message->CopyFrom( _messages.front() ); 
            _messages.pop();
            res = true;
        }
        _mutex.unlock();
    }
    return res;
}

void Udp_message_manager::send_message(
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

