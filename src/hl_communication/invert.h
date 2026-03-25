#pragma once
#include <hl_communication/wrapper.pb.h>

namespace hl_communication
{
void invertField(hl_communication::RobotMsg* robot_msg);

/**
 * Invert the side of the angle message (x-axis toward left or right of team area)
 */
void invertPosition(PositionDistribution* position);

/**
 * Invert the side of the angle message (x-axis toward left or right of team area)
 */
double invertAngle(double position);

/**
 * Invert the side of the provided pose message (x-axis toward left or right of team area)
 */
void invertPose(PoseDistribution* pose);
}