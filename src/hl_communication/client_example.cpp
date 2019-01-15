#include <iostream>
#include <hl_communication/message_manager.h>
#include <hl_communication/udp_message_manager.h>
#include <unistd.h>

int main(){
    hl_communication::Udp_message_manager udp_receiver(12345, -1);
 
    hl_communication::GameMsg msg;
    while(true){
        bool msg_received = udp_receiver.receive_message(&msg);
        if( msg_received ){
            assert( msg.has_robot_msg() );
            assert( msg.robot_msg()->has_robot_id() );
            assert( msg.robot_msg()->robot_id()->has_team_id() );
            assert( msg.robot_msg()->robot_id()->has_robot_id() );
           
            std::cout << "Receive Messge : " << std::endl; 
            std::cout << "robot id : " << msg.robot_msg().robot_id().robot_id()  << std::endl;
            std::cout << "team id : " << msg.robot_msg().robot_id().team_id()  << std::endl;
        }
    }
}
