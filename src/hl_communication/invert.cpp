#include "invert.h"
#include <cmath>


namespace hl_communication
{


void invertPosition(Position2d* position)
{
  position->set_x(-position->x());
  position->set_y(-position->y());
}


double invertAngle(double angle)
{
  double alpha = angle + M_PI;
  if (alpha > M_PI)
  {
    alpha -= 2 * M_PI;
  }
  return alpha;
}


void invertPose(Pose2d* pose)
{
  if (pose->has_position())
  {
    invertPosition(pose->mutable_position());
  }
  if (pose->has_dir())
  {
    pose->set_dir(invertAngle(pose->dir()));
  }
}

void invertField(hl_communication::Perception* perception)
{
  for (int idx = 0; idx < perception->self_in_field_size(); idx++)
  {
    invertPose(perception->mutable_self_in_field(idx)->mutable_pose());
  }
}

void invertField(hl_communication::Intention* intention)
{
  //if (intention->has_target_pose_in_field())
  //  invertPose(intention->mutable_target_pose_in_field());
  //for (int idx = 0; idx < intention->waypoints_in_field_size(); idx++)
  //{
  //  invertPose(intention->mutable_waypoints_in_field(idx));
  //}
  if (intention->has_kick_target_in_field())
    invertPosition(intention->mutable_kick_target_in_field());
}

void invertField(hl_communication::Captain* captain)
{
  if (captain->has_ball())
  {
    invertPosition(captain->mutable_ball()->mutable_position());
  }
  for (int i = 0; i < captain->opponents_size(); i++)
  {
    invertPose(captain->mutable_opponents(i)->mutable_pose());
  }
  for (int i = 0; i < captain->orders_size(); i++)
  {
    StrategyOrder* order = captain->mutable_orders(i);
    if (order->has_target_pose())
    {
      invertPose(order->mutable_target_pose());
    }
  }
}


void invertField(hl_communication::RobotMsg* robot_msg)
{
  if (robot_msg->has_perception())
    invertField(robot_msg->mutable_perception());
  if (robot_msg->has_intention())
    invertField(robot_msg->mutable_intention());
  if (robot_msg->has_captain())
    invertField(robot_msg->mutable_captain());
}

}