#include <iostream>
#include <hl_communication/message_manager.h>
#include <hl_communication/udp_message_manager.h>
#include <hl_communication/utils.h>
#include <unistd.h>

int main(int argc, char ** argv){
  int port_read = 12345;
  if (argc >= 2) {
    port_read = std::stoi(argv[1]);
  }
  std::cout << "Creating UDP Listener on port: " << port_read << std::endl;
  hl_communication::UDPMessageManager udp_receiver(port_read, -1);
 
  hl_communication::GameMsg msg;
  while(true){
    bool msg_received = udp_receiver.receiveMessage(&msg);
    if (!msg_received) {
      usleep(2000);
    } else {
      assert(msg.has_identifier());
      assert(msg.identifier()->has_src_ip());
      assert(msg.identifier()->has_port());
      if (msg.has_robot_msg()) {
        assert(msg.robot_msg()->has_robot_id());
        assert(msg.identifier()->has_packet_no());
        assert(msg.robot_msg()->robot_id()->has_team_id());
        assert(msg.robot_msg()->robot_id()->has_robot_id());

        std::cout << "Receive Message : " << std::endl; 
        std::cout << "robot id : " << msg.robot_msg().robot_id().robot_id()  << std::endl;
        std::cout << "team id : " << msg.robot_msg().robot_id().team_id()  << std::endl;
        unsigned int ip = msg.identifier().src_ip();
        std::cout << "Packet no : " << msg.identifier().packet_no()  << std::endl;
        std::cout << "Src port : " << msg.identifier().src_port()  << std::endl;
        std::cout << "Src ip (int64): "  << ip << std::endl;
        std::cout << "Src ip (human): "  << hl_communication::ipToString(ip) << std::endl;
        std::cout << std::endl;
      } else if (msg.has_gc_msg()) {
        std::cout << msg.DebugString() << std::endl;
      } else {
        std::cerr << "Message received which has neither robot msg nor gc message" << std::endl;
      }
    }
  }
}
