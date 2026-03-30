#include <hl_communication/robot_msg_utils.h>
#include <hl_communication/udp_message_manager.h>
//#include <hl_communication/utils.h>

#include <cmath>

namespace hl_communication
{

std::string action2str(Action a)
{
  switch(a)
  {
    case Action::POSITIONING:
      return "positioning";
    case Action::GOING_TO_KICK:
      return "going to kick";
    case Action::SEARCHING_BALL:
      return "searching ball";
    default:
      return "unknown";
  }
}

int getRobotId(const RobotMsg& msg)
{
  return msg.robot_id().robot_id();
}

int getTeamId(const RobotMsg& msg)
{
  return msg.robot_id().team_id();
}

uint64_t getAge(const RobotMsg& msg)
{
  return getTimeStamp() - msg.time_stamp();
}

Position2d fieldFromSelf(const Pose2d& robot_in_field, const Position2d& pos_in_self)
{
  double robot_x = robot_in_field.position().x();
  double robot_y = robot_in_field.position().y();
  double robot_dir = robot_in_field.dir();
  double pos_x = pos_in_self.x();
  double pos_y = pos_in_self.y();
  Position2d result;
  result.set_x(robot_x + cos(robot_dir) * pos_x - sin(robot_dir) * pos_y);
  result.set_y(robot_y + sin(robot_dir) * pos_x + cos(robot_dir) * pos_y);
  return result;
}

}
