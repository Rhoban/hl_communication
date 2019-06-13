#include <hl_communication/labelling_utils.h>

#include <hl_communication/utils.h>

namespace hl_communication
{
bool sameBall(const BallMsg& msg1, const BallMsg& msg2)
{
  return msg1.has_ball_id() && msg2.has_ball_id() && msg1.ball_id() == msg2.ball_id();
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
}

}  // namespace hl_communication
