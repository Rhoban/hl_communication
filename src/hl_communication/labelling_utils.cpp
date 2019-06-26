#include <hl_communication/labelling_utils.h>

#include <hl_communication/utils.h>

namespace hl_communication
{
bool sameBall(const BallMsg& msg1, const BallMsg& msg2)
{
  return msg1.has_ball_id() && msg2.has_ball_id() && msg1.ball_id() == msg2.ball_id();
}

bool sameRobot(const RobotMessage& msg1, const RobotMessage& msg2)
{
  if (!msg1.has_robot_id() || !msg2.has_robot_id())
    return false;
  const RobotIdentifier& id1 = msg1.robot_id();
  const RobotIdentifier& id2 = msg2.robot_id();
  return id1.robot_id() == id2.robot_id() && id1.team_id() && id2.team_id();
}

void exportLabel(const LabelMsg& src, LabelMsg* dst)
{
  // checking frame indices
  if (!src.has_frame_index())
  {
    throw std::logic_error(HL_DEBUG + "src has no frame index");
  }
  uint32_t frame_index = src.frame_index();
  if (!dst->has_frame_index())
  {
    dst->set_frame_index(frame_index);
  }
  else if (dst->frame_index() != frame_index)
  {
    throw std::logic_error(HL_DEBUG + "dst (" + std::to_string(dst->frame_index()) + ") and src (" +
                           std::to_string(frame_index) + ") have different frame indices");
  }
  // Treat balls
  for (const BallMsg& new_ball : src.balls())
  {
    bool replaced = false;
    for (int idx = 0; idx < dst->balls_size(); idx++)
    {
      BallMsg* old_ball = dst->mutable_balls(idx);
      if (sameBall(new_ball, *old_ball))
      {
        replaced = true;
        old_ball->CopyFrom(new_ball);
      }
    }
    if (!replaced)
    {
      dst->add_balls()->CopyFrom(new_ball);
    }
  }
  // TODO: Treat robots
  for (const RobotMessage& new_robot : src.robots())
  {
    bool replaced = false;
    for (int idx = 0; idx < dst->robots_size(); idx++)
    {
      RobotMessage* old_robot = dst->mutable_robots(idx);
      if (sameRobot(new_robot, *old_robot))
      {
        replaced = true;
        old_robot->CopyFrom(new_robot);
      }
    }
    if (!replaced)
    {
      dst->add_robots()->CopyFrom(new_robot);
    }
  }
  // Export field matches
  for (const Match2D3DMsg& new_match : src.field_matches())
  {
    // OPTIONAL: if two points have exactly the same coordinate, we could do something here
    dst->add_field_matches()->CopyFrom(new_match);
  }
}

}  // namespace hl_communication
