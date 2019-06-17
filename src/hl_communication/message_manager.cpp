#include <hl_communication/message_manager.h>

#include <fstream>
#include <iostream>
#include <sstream>

namespace hl_communication
{
std::map<uint32_t, std::vector<RobotMsg>> MessageManager::Status::getRobotsByTeam() const
{
  std::map<uint32_t, std::vector<RobotMsg>> messages_by_team;
  for (const auto& entry : robot_messages)
  {
    messages_by_team[entry.first.team_id()].push_back(entry.second);
  }
  return messages_by_team;
}

MessageManager::MessageManager() : clock_offset(0), auto_discover_ports(false)
{
}

MessageManager::MessageManager(const std::string& file_path) : MessageManager()
{
  loadMessages(file_path);
}

MessageManager::MessageManager(int port_read, bool auto_discover) : MessageManager()
{
  auto_discover_ports = auto_discover;
  openReceiver(port_read);
}

MessageManager::MessageManager(const std::vector<int>& ports) : MessageManager()
{
  for (int p : ports)
  {
    openReceiver(p);
  }
}

void MessageManager::update()
{
  for (auto& entry : udp_receivers)
  {
    GameMsg msg;
    while (entry.second->receiveMessage(&msg))
    {
      push(msg);
    }
  }
}

void MessageManager::saveMessages(const std::string& path)
{
  GameMsgCollection collection = buildGameMsgCollection();
  std::cout << "Serializing a collection of " << collection.messages_size() << " messages" << std::endl;
  std::ofstream out(path);
  if (!out.good())
  {
    throw std::runtime_error("MessageManager: Failed to open file '" + path + "'");
  }
  if (!collection.SerializeToOstream(&out))
  {
    throw std::runtime_error("MessageManager: Failed to write in '" + path + "'");
  }
}

uint64_t MessageManager::getStart() const
{
  uint64_t min_ts = std::numeric_limits<uint64_t>::max();
  if (gc_messages.size() > 0)
  {
    min_ts = std::min(min_ts, gc_messages.begin()->first);
  }
  for (const auto& robot_collection : messages_by_robot)
  {
    if (robot_collection.second.size() > 0)
    {
      min_ts = std::min(min_ts, robot_collection.second.begin()->first);
    }
  }
  return min_ts;
}

uint64_t MessageManager::getEnd() const
{
  uint64_t max_ts = 0;
  if (gc_messages.size() > 0)
  {
    max_ts = std::max(max_ts, gc_messages.rbegin()->first);
  }
  for (const auto& robot_collection : messages_by_robot)
  {
    if (robot_collection.second.size() > 0)
    {
      max_ts = std::max(max_ts, robot_collection.second.rbegin()->first);
    }
  }
  return max_ts;
}

MessageManager::Status MessageManager::getStatus(uint64_t time_stamp, bool system_clock) const
{
  if (system_clock)
  {
    time_stamp -= clock_offset;
  }
  Status status;
  for (const auto& robot_entry : messages_by_robot)
  {
    auto it = robot_entry.second.upper_bound(time_stamp);
    if (it == robot_entry.second.end())
      it--;
    if (it->first > time_stamp)
    {
      // Do not include robots which have no data prior to time_stamp
      if (it == robot_entry.second.begin())
        continue;
      it--;
    }
    status.robot_messages[robot_entry.first] = it->second;
  }
  if (gc_messages.size() > 0)
  {
    auto it = gc_messages.upper_bound(time_stamp);
    if (it == gc_messages.end())
      it--;
    if (it->first > time_stamp)
    {
      // There are no data prior to time_stamp
      if (it == gc_messages.begin())
        return status;
      it--;
    }
    status.gc_message = it->second;
  }
  return status;
}

MessageManager::Status MessageManager::getStatus(uint64_t time_stamp, uint64_t history_length, bool system_clock) const
{
  if (system_clock)
  {
    time_stamp -= clock_offset;
  }
  Status status;
  for (const auto& robot_entry : messages_by_robot)
  {
    auto it = robot_entry.second.upper_bound(time_stamp);
    if (it == robot_entry.second.end())
      it--;
    if (it->first > time_stamp)
    {
      // Do not include robots which have no data prior to time_stamp
      if (it == robot_entry.second.begin())
        continue;
      it--;
    }
    if (it->first >= (time_stamp - history_length))
    {
      status.robot_messages[robot_entry.first] = it->second;
    }
  }
  if (gc_messages.size() > 0)
  {
    auto it = gc_messages.upper_bound(time_stamp);
    if (it == gc_messages.end())
      it--;
    if (it->first > time_stamp)
    {
      // There are no data prior to time_stamp
      if (it == gc_messages.begin())
        return status;
      it--;
    }
    if (it->first >= (time_stamp - history_length))
    {
      status.gc_message = it->second;
    }
  }
  return status;
}

void MessageManager::push(const RobotMsg& msg)
{
  if (!msg.has_robot_id())
  {
    throw std::runtime_error("MessageManager can only handle identified RobotMsg");
  }
  if (!msg.has_utc_time_stamp())
  {
    throw std::runtime_error("MessageManager can only handle utc time_stamped RobotMsg");
  }
  messages_by_robot[msg.robot_id()][msg.utc_time_stamp()] = msg;
}

void MessageManager::push(const GCMsg& msg)
{
  if (!msg.has_utc_time_stamp())
  {
    throw std::runtime_error("MessageManager can only handle utc time_stamped GCMsg");
  }
  gc_messages[msg.utc_time_stamp()] = msg;
  if (auto_discover_ports)
  {
    for (const GCTeamMsg& team_msg : msg.teams())
    {
      int team_id = team_msg.team_number();
      int port = getDefaultTeamPort(team_id);
      if (udp_receivers.count(port) == 0)
      {
        openReceiver(port);
      }
    }
  }
}

void MessageManager::openReceiver(int port)
{
  if (udp_receivers.count(port) != 0)
    throw std::logic_error(HL_DEBUG + "Trying to open two receivers on port: " + std::to_string(port));
  udp_receivers[port] = std::unique_ptr<UDPMessageManager>(new UDPMessageManager(port, -1));
}

void MessageManager::push(const GameMsg& msg)
{
  // Avoid to store twice duplicated message and print warning
  if (received_messages.count(msg.identifier()) > 0)
  {
    std::cerr << "Duplicated message received" << std::endl;
    // TODO: show message identifier
    return;
  }
  received_messages[msg.identifier()] = msg;
  if (msg.has_robot_msg())
  {
    push(msg.robot_msg());
  }
  else if (msg.has_gc_msg())
  {
    SourceIdentifier source_id;
    source_id.src_ip = msg.identifier().src_ip();
    source_id.src_port = msg.identifier().src_port();
    gc_sources.insert(source_id);
    if (gc_sources.size() > 1)
    {
      std::ostringstream oss;
      oss << HL_DEBUG << " more than one different source of GameController messages has been found"
          << " sources: " << std::endl;
      for (const SourceIdentifier& id : gc_sources)
      {
        oss << "\t"
            << "ip: " << ipToString(id.src_ip) << " port: " << id.src_port << std::endl;
      }
      std::cerr << oss.str();
    }
    push(msg.gc_msg());
  }
  else
  {
    throw std::runtime_error("Failed to read GameMsg, not a RobotMsg neither a GCMsg");
  }
}

void MessageManager::push(const GameMsgCollection& collection)
{
  for (const GameMsg& msg : collection.messages())
  {
    push(msg);
  }
}

void MessageManager::loadMessages(const std::string& file_path)
{
  std::ifstream in(file_path, std::ios::binary);
  if (!in.good())
  {
    throw std::runtime_error("Failed to open file at '" + file_path + "'");
  }
  GameMsgCollection messages_collection;
  if (!messages_collection.ParseFromIstream(&in))
  {
    throw std::runtime_error("Failed to parse messages in file '" + file_path + "'");
  }
  if (messages_collection.has_time_offset())
  {
    setOffset(messages_collection.time_offset());
  }
  std::cout << "Pushing " << messages_collection.messages_size() << " messages in MM" << std::endl;
  for (const GameMsg msg : messages_collection.messages())
  {
    push(msg);
  }
}

GameMsgCollection MessageManager::buildGameMsgCollection() const
{
  GameMsgCollection result;
  for (const auto& entry : received_messages)
  {
    result.add_messages()->CopyFrom(entry.second);
  }
  return result;
}

void MessageManager::setOffset(int64_t new_offset)
{
  clock_offset = new_offset;
}

int64_t MessageManager::getOffset() const
{
  return clock_offset;
}

bool operator<(const MessageManager::SourceIdentifier& id1, const MessageManager::SourceIdentifier& id2)
{
  if (id1.src_ip != id2.src_ip)
  {
    return id1.src_ip < id2.src_ip;
  }
  return id1.src_port < id2.src_port;
}

}  // namespace hl_communication
