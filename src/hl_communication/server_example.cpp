#include <iostream>
#include <hl_communication/message_manager.h>
#include <hl_communication/udp_message_manager.h>
#include <unistd.h>

int main(){
    hl_communication::GameMsg msg;

    hl_communication::Udp_message_manager udp_sender(-1, 12345);
    
    int i=0;
    while(true){
        msg.mutable_robot_msg()->mutable_robot_id()->set_robot_id( i );
        msg.mutable_robot_msg()->mutable_robot_id()->set_team_id( 100 + i );
        
        std::cout << "Send message : " << msg.DebugString() << std::endl;
        udp_sender.send_message( msg ); 
        
        usleep(1000000.0);
        i++;
    }
}
