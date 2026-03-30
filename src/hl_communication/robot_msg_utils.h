#pragma once

#include <hl_communication/wrapper.pb.h>

namespace hl_communication
{

std::string action2str(Action a);

int getRobotId(const RobotMsg& msg);

int getTeamId(const RobotMsg& msg);

/**
 * Age of the message in micro-seconds
 */
uint64_t getAge(const RobotMsg& msg);

/**
 * Convert the given position distribution in self referential to world referential.
 * DISCLAIMER: currently, the resulting PositionDistribution does not take into account uncertainties
 */
Position2d fieldFromSelf(const Pose2d& robot_in_field, const Position2d& pos_in_self);

}
