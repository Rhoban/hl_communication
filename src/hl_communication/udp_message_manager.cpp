#include <hl_communication/udp_message_manager.h>

#include <hl_communication/game_controller_utils.h>
#include <hl_communication/utils.h>

#include <chrono>
#include <iostream>

#define PACKET_MAX_SIZE 10000

using namespace std::chrono;

namespace hl_communication {

int getDefaultTeamPort(int team_id) {
  return 35000 + team_id;
}

UDPMessageManager::UDPMessageManager(int port_read, int port_write){
  packet_sent_no = 0;
  packet_gc_no = 0;
  continue_to_run = true;
  port_read = port_read;
  port_write = port_write;
  broadcaster.reset(new hl_communication::UDPBroadcast(port_read, port_write));
  if(port_read != -1){
    broadcaster->openRead();
    thread.reset(new std::thread([this](){this->run();})); 
  }
  if(port_write != -1){
    broadcaster->openWrite();
  }
}

void UDPMessageManager::run(){
  //Receiving informations
  char data[PACKET_MAX_SIZE];
  size_t len;
  uint64_t src_address;
  uint32_t src_port;
  hl_communication::GameMsg game_msg;
  while (continue_to_run) {
    len = PACKET_MAX_SIZE;
    if(! broadcaster->checkMessage(data, &len, &src_address, &src_port)) continue;
    if (len >= PACKET_MAX_SIZE) {
      std::cout << "Packet are too long !" << std::endl;
      continue;
    }
    game_msg.Clear();
    std::string string_data(data, len);
    uint64_t time_stamp = getTimeStamp();
    GameState game_state;
    // Disabling error message when ParseFromString fails
    google::protobuf::LogSilencer silencer;
    if(game_msg.ParseFromString(string_data)){
    } else if (game_state.updateFromMessage(data)) {
      game_state.exportToGCMsg(game_msg.mutable_gc_msg());
      game_msg.mutable_identifier()->set_packet_no(packet_gc_no);
      packet_gc_no++;
    } else {
      std::cerr << "Invalid format for a packet of size: " << len << std::endl;
      continue;
    }

    game_msg.mutable_identifier()->set_src_ip(src_address);
    game_msg.mutable_identifier()->set_src_port(ntohs(src_port));
    //Assign reception timestamp
    if(game_msg.has_gc_msg()){
      game_msg.mutable_gc_msg()->set_time_stamp(time_stamp);
    }
    if(game_msg.has_robot_msg()){
      game_msg.mutable_robot_msg()->set_time_stamp(time_stamp);
    }
      
    mutex.lock();
    messages.push(game_msg);
    mutex.unlock();
  }
}

UDPMessageManager::~UDPMessageManager(){
  continue_to_run = false;
  if(port_read != -1){
    thread->join();
  }
  if(port_write != -1){
    broadcaster->closeWrite();
  }
}

 
bool UDPMessageManager::receiveMessage(hl_communication::GameMsg * message){
  bool res = false;
  if(port_read != -1){
    mutex.lock();
    if(!messages.empty()){
      message->Clear();
      message->CopyFrom(messages.front()); 
      messages.pop();
      res = true;
    }
    mutex.unlock();
  }
  return res;
}

void UDPMessageManager::sendMessage(const hl_communication::GameMsg & message){
  std::string raw_message;
  if(!message.SerializeToString(&raw_message)){
    std::cerr << "Invalid Message !" << std::endl;
    return;
  }
  broadcaster->broadcastMessage(raw_message.c_str(), raw_message.size());
}

void UDPMessageManager::sendMessage(hl_communication::GameMsg * message){
  message->mutable_identifier()->set_packet_no(packet_sent_no);
  packet_sent_no++; 
  sendMessage(*message);
}

}

