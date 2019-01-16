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
            assert( msg.has_identifier() );
            assert( msg.robot_msg()->has_robot_id() );
            assert( msg.identifier()->has_packet_no() );
            assert( msg.identifier()->has_src_ip() );
            assert( msg.identifier()->has_port() );
            assert( msg.robot_msg()->robot_id()->has_team_id() );
            assert( msg.robot_msg()->robot_id()->has_robot_id() );
           
            std::cout << "Receive Message : " << std::endl; 
            std::cout << "robot id : " << msg.robot_msg().robot_id().robot_id()  << std::endl;
            std::cout << "team id : " << msg.robot_msg().robot_id().team_id()  << std::endl;
            std::cout << "Packet no : " << msg.identifier().packet_no()  << std::endl;
            std::cout << "Src port : " << msg.identifier().src_port()  << std::endl;
            unsigned int ip = msg.identifier().src_ip();
            std::cout 
                << "Src ip : " 
                << ( ip>>24 &0xFF )  << "."
                << ( ip>>16 &0xFF )  << "."
                << ( ip>>8  &0xFF )  << "."
                << ( ip     &0xFF )  
                << std::endl;
            std::cout << std::endl;
        }
    }
}
