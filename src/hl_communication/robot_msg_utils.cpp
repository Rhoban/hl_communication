#include <hl_communication/robot_msg_utils.h>
#include <hl_communication/utils.h>

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
    case Action::KICKING:
      return "kicking";
    case Action::GOING_TO_DRIBBLE:
      return "going to dribble";
    case Action::DRIBBLING:
      return "dribbling";
    case Action::WAITING:
      return "waiting";
    case Action::SEARCHING_BALL:
      return "searching ball";
    case Action::INACTIVE:
      return "inactive";
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

PositionDistribution fieldFromSelf(const PoseDistribution& robot_in_field, const PositionDistribution& pos_in_self)
{
  double robot_x = robot_in_field.position().x();
  double robot_y = robot_in_field.position().y();
  double robot_dir = robot_in_field.dir().mean();
  double pos_x = pos_in_self.x();
  double pos_y = pos_in_self.y();
  PositionDistribution result;
  result.set_x(robot_x + cos(robot_dir) * pos_x - sin(robot_dir) * pos_y);
  result.set_y(robot_y + sin(robot_dir) * pos_x + cos(robot_dir) * pos_y);
  return result;
}

double getBallDistance(const RobotMsg& msg)
{
  const PositionDistribution& pos = msg.perception().ball_in_self();
  double x = pos.x();
  double y = pos.y();
  return std::sqrt(pow(x,2) + pow(y,2));
}

PositionDistribution getBallInField(const RobotMsg& msg)
{
  return fieldFromSelf(msg.perception().self_in_field(0).pose(), msg.perception().ball_in_self());
}
}
