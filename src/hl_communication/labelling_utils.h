#pragma once

#include <hl_communication/labelling.pb.h>

namespace hl_communication
{
/**
 * Return true if the two messages have the same id provided
 */
bool sameBall(const BallMsg& msg1, const BallMsg& msg2);

/**
 * Return true if the two messages have the same id provided
 */
bool sameRobot(const RobotMessage& msg1, const RobotMessage& msg2);

/**
 * Export the labelling stored in src to dst, overriding existing data if required
 * throws an error if src and dst have different frame indices
 */
void exportLabel(const LabelMsg& src, LabelMsg* dst);

}  // namespace hl_communication
