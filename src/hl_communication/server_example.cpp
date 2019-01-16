#include <iostream>
#include <hl_communication/message_manager.h>
#include <hl_communication/udp_message_manager.h>
#include <unistd.h>

int main(){
  hl_communication::GameMsg msg;

  hl_communication::UDPMessageManager udp_sender(-1, 12345);
    
  int i=0;
  while(true){
    msg.Clear();
    msg.mutable_robot_msg()->mutable_robot_id()->set_robot_id(100+i);
    msg.mutable_robot_msg()->mutable_robot_id()->set_team_id(1000+i);
        
    udp_sender.sendMessage(&msg); 
    std::cout << "Send message : " << msg.DebugString() << std::endl;
    std::cout << std::endl;
        
    usleep(1000000.0);
    i++;
  }
}
