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
PositionDistribution fieldFromSelf(const PoseDistribution& robot_in_field, const PositionDistribution& pos_in_self);

double getBallDistance(const RobotMsg& msg);

/**
 * Uses the ball in the field 
 */
PositionDistribution getBallInField(const RobotMsg& robot_msg);

}
