#include <hl_communication/message_manager.h>

#include <hl_communication/utils.h>

#include <fstream>
#include <iostream>
#include <sstream>

namespace hl_communication
{

bool operator<(const RobotIdentifier & id1, const RobotIdentifier & id2) {
  return (id1.team_id() == id2.team_id() && id1.robot_id() < id2.robot_id()) ||
    id1.team_id() < id2.team_id();
}

bool operator<(const MsgIdentifier & id1, const MsgIdentifier & id2) {
  if (!id1.has_src_ip() || !id2.has_src_ip() || !id1.has_src_port() || !id2.has_src_port()) {
    throw std::runtime_error("Incomplete message identifier");
  }
  if (id1.src_ip() != id2.src_ip()) {
    return id1.src_ip() < id2.src_ip();
  }
  if (id1.src_port() != id2.src_port()) {
    return id1.src_port() < id2.src_port();
  }
  return id1.packet_no() < id2.packet_no();
}

MessageManager::MessageManager(const std::string & file_path) {
  loadMessages(file_path);
}

MessageManager::MessageManager(int port_read)
{
  udp_receivers.push_back(std::unique_ptr<UDPMessageManager>(new UDPMessageManager(port_read, -1)));
}

MessageManager::MessageManager(const std::vector<int> & ports)
{
  for (int p : ports) {
    udp_receivers.push_back(std::unique_ptr<UDPMessageManager>(new UDPMessageManager(p, -1)));
  }
}

void MessageManager::update() {
  for (size_t idx = 0; idx < udp_receivers.size(); idx++) {
    GameMsg msg;
    while(udp_receivers[idx]->receiveMessage(&msg)) {
      push(msg);
    }
  }
}

void MessageManager::saveMessages(const std::string & path) {
  GameMsgCollection collection = buildGameMsgCollection();
  std::cout << "Serializing a collection of " << collection.messages_size() << " messages" << std::endl;
  std::ofstream out(path);
  if (!out.good()) {
    throw std::runtime_error("MessageManager: Failed to open file '" + path + "'");
  }
  if (!collection.SerializeToOstream(&out)) {
    throw std::runtime_error("MessageManager: Failed to write in '" + path + "'");
  }
}

uint64_t MessageManager::getStart() const {
  uint64_t min_ts = std::numeric_limits<uint64_t>::max();
  if (gc_messages.size() > 0) {
    min_ts = std::min(min_ts, gc_messages.begin()->first);
  }
  for (const auto & robot_collection : messages_by_robot) {
    if (robot_collection.second.size() > 0) {
      min_ts = std::min(min_ts, robot_collection.second.begin()->first);
    }
  }
  return min_ts;
}

MessageManager::Status MessageManager::getStatus(uint64_t time_stamp) const {
  Status status;
  for (const auto & robot_entry : messages_by_robot) {
    std::cout << "-> Checking messages from robot " << robot_entry.first.robot_id()
                << " from team " << robot_entry.first.team_id() << std::endl;
    auto it = robot_entry.second.upper_bound(time_stamp);
    // Do not include robots which have no data prior to time_stamp
    if (it == robot_entry.second.end())
      continue;
    if (it->first > time_stamp) {
      if (it == robot_entry.second.begin())
        continue;
      it--;
    }
    status.robot_messages[robot_entry.first] = it->second;
  }
  auto it = gc_messages.upper_bound(time_stamp);
  if (it == gc_messages.end())
    return status;
  if (it->first > time_stamp) {
    if (it == gc_messages.begin())
      return status;
    it--;
  }
  status.gc_message = it->second;
  return status;
}

void MessageManager::push(const RobotMsg & msg) {
  if (!msg.has_robot_id()) {
    throw std::runtime_error("MessageManager can only handle identified RobotMsg");
  }
  if (!msg.has_time_stamp()) {
    throw std::runtime_error("MessageManager can only handle time_stamped RobotMsg");
  }
  messages_by_robot[msg.robot_id()][msg.time_stamp()] = msg;
}

void MessageManager::push(const GCMsg & msg) {
  if (!msg.has_time_stamp()) {
    throw std::runtime_error("MessageManager can only handle time_stamped GCMsg");
  }
  gc_messages[msg.time_stamp()] = msg;
}

void MessageManager::push(const GameMsg & msg) {
  // Avoid to store twice duplicated message and print warning
  if (received_messages.count(msg.identifier()) > 0) {
    std::cerr << "Duplicated message received" << std::endl;
    //TODO: show message identifier
    return;
  }
  received_messages[msg.identifier()] = msg;
  if (msg.has_robot_msg()) {
    push(msg.robot_msg());
    
  } else if (msg.has_gc_msg()) {
    SourceIdentifier source_id;
    source_id.src_ip = msg.identifier().src_ip();
    source_id.src_port = msg.identifier().src_port();
    gc_sources.insert(source_id);
    if (gc_sources.size() > 1) {
      std::ostringstream oss;
      oss << HL_DEBUG << " more than one different source of GameController messages has been found"
          << " sources: " << std::endl;
      for (const SourceIdentifier & id : gc_sources) {
        oss << "\t" << "ip: " << ipToString(id.src_ip) << " port: " << id.src_port << std::endl;
      }
      std::cerr << oss.str();
    }
    push(msg.gc_msg());
  } else {
    throw std::runtime_error("Failed to read GameMsg, not a RobotMsg neither a GCMsg");
  }
}

void MessageManager::push(const GameMsgCollection & collection) {
  for (const GameMsg & msg : collection.messages()) {
    push(msg);
  }
}

void MessageManager::loadMessages(const std::string & file_path) {
  std::ifstream in(file_path, std::ios::binary);
  if (!in.good()) {
    throw std::runtime_error("Failed to open file at '" + file_path + "'");
  }
  GameMsgCollection messages_collection;
  if (!messages_collection.ParseFromIstream(&in)) {
    throw std::runtime_error("Failed to parse messages in file '" + file_path + "'");
  }
  std::cout << "Pushing " << messages_collection.messages_size() << " messages in MM" << std::endl;
  for (const GameMsg msg : messages_collection.messages()) {
    push(msg);
  }
}

GameMsgCollection MessageManager::buildGameMsgCollection() const {
  GameMsgCollection result;
  for (const auto & entry : received_messages) {
    result.add_messages()->CopyFrom(entry.second);
  }
  return result;
}

bool operator<(const MessageManager::SourceIdentifier & id1,
               const MessageManager::SourceIdentifier & id2)
{
  if (id1.src_ip != id2.src_ip) {
    return id1.src_ip < id2.src_ip;
  }
  return id1.src_port < id2.src_port;
}

}
