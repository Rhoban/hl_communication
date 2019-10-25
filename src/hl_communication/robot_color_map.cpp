#include <hl_communication/robot_color_map.h>

namespace hl_communication
{
RobotColorMap::RobotColorMap()
{
}

TeamColor RobotColorMap::getColor(const RobotIdentifier& id) const
{
  if (count(id) == 0)
    return TeamColor::UNKNOWN;
  return at(id);
}

void RobotColorMap::pushColor(const RobotIdentifier& id, TeamColor c)
{
  TeamColor current_color = getColor(id);
  if (current_color != c)
  {
    if (current_color == TeamColor::UNKNOWN)
      operator[](id) = c;
    else
      operator[](id) = TeamColor::CONFLICT;
  }
}

}  // namespace hl_communication
