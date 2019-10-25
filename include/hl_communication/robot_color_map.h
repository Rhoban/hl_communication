#pragma once

#include <hl_communication/utils.h>

namespace hl_communication
{
class RobotColorMap : public std::map<RobotIdentifier, TeamColor>
{
public:
  RobotColorMap();

  TeamColor getColor(const RobotIdentifier& id) const;
  void pushColor(const RobotIdentifier& id, TeamColor c);
};

}  // namespace hl_communication
