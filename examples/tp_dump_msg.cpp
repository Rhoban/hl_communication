#include "hl_communication/position.pb.h"
#include "hl_communication/team_play.pb.h"
#include "hl_communication/wrapper.pb.h"
#include <iostream>
#include <hl_communication/udp_message_manager.h>
#include "rhoban_utils/timing/time_stamp.h"
#include <unistd.h>
#include <iomanip>

struct Stats
{
  std::vector<hl_communication::GameMsg> msgs;
  int totalBytes = 0;
};

struct Source
{
  uint64_t ip;
  uint32_t port;
  bool operator<(const Source& other) const
  {
    if (ip != other.ip)
    {
      return ip < other.ip;
    }
    return port < other.port;
  }
  rhoban_utils::TimeStamp firstMsg = rhoban_utils::TimeStamp::now();
};

int main(int argc, char** argv)
{
  int port_read = 10011;
  std::map<Source, Stats> statsPerRobot;
  if (argc >= 2)
  {
    port_read = std::stoi(argv[1]);
  }
  std::cout << "Creating UDP Listener on port: " << port_read << std::endl;
  hl_communication::UDPMessageManager udp_receiver(port_read, -1);

  rhoban_utils::TimeStamp printStats = rhoban_utils::TimeStamp::now();

  std::cout << std::setprecision(2) << std::fixed;
  hl_communication::GameMsg msg;
  while (true)
  {
    if ((rhoban_utils::TimeStamp::now().getTimeSec() - printStats.getTimeSec()) > 5)
    {
      printStats = rhoban_utils::TimeStamp::now();
      std::cout << "* STATS *" << std::endl;
      std::cout << "Total players/sources: " << statsPerRobot.size() << std::endl;
      for (auto& entry : statsPerRobot)
      {
        double duration = rhoban_utils::TimeStamp::now().getTimeSec() - entry.first.firstMsg.getTimeSec();
        std::cout << "Player (" << entry.first.ip << ":" << entry.first.port << ") - " << entry.second.msgs.size()
                  << " messages, " << entry.second.totalBytes
                  << " bytes, msg/s: " << entry.second.msgs.size() / duration
                  << ", bytes/s: " << entry.second.totalBytes / duration << std::endl;
      }
    }

    bool msg_received = udp_receiver.receiveMessage(&msg);
    if (!msg_received)
    {
      usleep(2000);
    }
    else
    {
      if (msg.has_identifier())
      {
        Source src;
        src.ip = msg.identifier().src_ip();
        src.port = msg.identifier().src_port();
        std::cout << "Received message from " << src.ip << ":" << src.port << std::endl;
        statsPerRobot[src].msgs.push_back(msg);
        statsPerRobot[src].totalBytes += msg.ByteSizeLong();
      }
      else
      {
        std::cout << "\x1b[31mERROR: Received message without identifier\x1b[0m" << std::endl;
      }
      if (!msg.has_robot_msg())
      {
        std::cout << "\x1b[31mERROR: Message does not contain a robot message!\x1b[0m" << std::endl;
      }
      else
      {
        const hl_communication::RobotMsg& robot_msg = msg.robot_msg();
        std::cout << "Robot " << robot_msg.robot_id().team_id() << ":" << robot_msg.robot_id().robot_id() << std::endl;
        // PERCEPTION
        if (robot_msg.has_perception())
        {
          const hl_communication::Perception& perception = robot_msg.perception();
          if (perception.has_ball_in_self())
          {
            const hl_communication::Position2d& ball = perception.ball_in_self();
            std::cout << "\tBall position: (" << ball.x() << ", " << ball.y()
                      << ") (quality: " << perception.ball_quality() << ")" << std::endl;
          }
          else
          {
            std::cout << "\tNo ball in perception" << std::endl;
          }
          if (perception.has_self_in_field())
          {
            const hl_communication::Pose2d& self = perception.self_in_field();
            std::cout << "\tSelf position: (" << self.position().x() << ", " << self.position().y()
                      << "), dir: " << self.dir() << " quality: " << perception.field_quality() << std::endl;
          }
          else
          {
            std::cout << "\tNo self in perception" << std::endl;
          }
          if (perception.robots_field_size() > 0)
          {
            std::cout << "\tRobots in perception:" << std::endl;
            for (int i = 0; i < perception.robots_field_size(); i++)
            {
              const hl_communication::Position2d& robot = perception.robots_field(i);
              std::cout << "\t - Robot " << i << ": (" << robot.x() << ", " << robot.y() << ")" << std::endl;
            }
          }
          else
          {
            std::cout << "\tNo robots in perception" << std::endl;
          }
          if (perception.has_kicked())
          {
            if (perception.kicked())
              std::cout << "\t\x1b[32mBall was kicked recently!\x1b[0m" << std::endl;
          }
          else
          {
            std::cout << "\tNo kicked information in perception" << std::endl;
          }
          if (perception.has_whistle_heard())
          {
            if (perception.whistle_heard())
              std::cout << "\t\x1b[32mWhistle heard recently!\x1b[0m" << std::endl;
          }
          else
          {
            std::cout << "\tNo whistle information in perception" << std::endl;
          }
        }
        else
        {
          std::cout << "No perception in robot message" << std::endl;
        }
        // TEAM PLAY
        if (robot_msg.has_team_play())
        {
          const hl_communication::TeamPlay& team_play = robot_msg.team_play();
          std::cout << "\tTeam play status: " << (team_play.isplaying() ? "Playing" : "Not playing") << std::endl;
          if (team_play.has_role())
          {
            std::cout << "\tRole: " << hl_communication::Role_Name(team_play.role()) << std::endl;
          }
          else
          {
            std::cout << "\tNo role information in team play" << std::endl;
          }
          if (team_play.has_status())
          {
            std::cout << "\tStatus: " << hl_communication::Status_Name(team_play.status()) << std::endl;
          }
          else
          {
            std::cout << "\tNo status information in team play" << std::endl;
          }
        }
        else
        {
          std::cout << "\tNo team play information in robot message" << std::endl;
        }
        // INTENTION
        if (robot_msg.has_intention())
        {
          const hl_communication::Intention& intention = robot_msg.intention();
          std::cout << "\tIntended action: " << intention.action_planned() << std::endl;
        }
        else
        {
          std::cout << "\tNo intention information in robot message" << std::endl;
        }
        // CAPTAIN
        if (robot_msg.has_captain())
        {
          const hl_communication::Captain& captain = robot_msg.captain();
          if (captain.has_ball())
          {
            const hl_communication::Position2d& ball = captain.ball().position();
            std::cout << "\tCaptain ball position: (" << ball.x() << ", " << ball.y()
                      << ") (votes: " << captain.ball().nb_votes() << ")" << std::endl;
          }
          else
          {
            std::cout << "\tNo ball in captain" << std::endl;
          }
          std::cout << "\tCaptain opponents: " << captain.opponents_size() << std::endl;
          for (int i = 0; i < captain.opponents_size(); i++)
          {
            const hl_communication::Position2d& opponent = captain.opponents(i).pose().position();
            std::cout << "\t - Opponent " << i << ": (" << opponent.x() << ", " << opponent.y()
                      << ") (votes: " << captain.opponents(i).nb_votes() << ")" << std::endl;
          }

          std::cout << "\tCaptain orders: " << captain.orders_size() << std::endl;
          for (int i = 0; i < captain.orders_size(); i++)
          {
            const hl_communication::StrategyOrder& order = captain.orders(i);
            std::cout << "\t - Order for robot " << order.robot_id()
                      << ": action=" << hl_communication::Action_Name(order.action()) << std::endl;
            if (order.has_target())
            {
              std::cout << "\t\ttarget=(x=" << order.target().position().x() << " y=" << order.target().position().y()
                        << " dir=" << order.target().dir() << ")";
            }
            std::cout << std::endl;
          }
        }
        else
        {
          std::cout << "\tNo captain information in robot message" << std::endl;
        }
        // print monitoring
        auto& m = robot_msg.monitoring();
        std::cout << "\tMonitoring: inGame=" << m.ingame() << " canMove=" << m.canmove() << " canKick=" << m.cankick()
                  << " canScore=" << m.canscore() << std::endl;
        std::cout << "\tstayOurField=" << m.stayourfield() << " ballClearing=" << m.ballclearing() << std::endl;
        std::cout << "\trefereePhase=" << m.refereephase() << " captainPlacer=" << m.captainplacer() << std::endl;
      }
    }
  }
  return 0;
}