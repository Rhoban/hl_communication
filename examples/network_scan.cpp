/**
 * network_scan --- surveillance reseau RoboCup
 *
 * Ecoute simultanement (boucle poll, single-thread) :
 *   - port 3838  : paquets RoboCupGameControlData  (GC -> robots)
 *   - port 10011 : messages protobuf GameMsg        (robots -> equipe)
 *
 * Affichage terminal rafraichi ~2x/s.
 */

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstring>
#include <deque>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "robocup_referee/RoboCupGameControlData.h"
#include "hl_communication/wrapper.pb.h"

static const char* RED = "\033[31m";
static const char* RESET = "\033[0m";
static const char* BOLD = "\033[1m";

static const double FREQ_WINDOW_SEC = 5.0;
static const int POLL_TIMEOUT_MS = 50;
static const double REFRESH_SEC = 0.5;

static volatile sig_atomic_t g_running = 1;
static void sigHandler(int)
{
  g_running = 0;
}

static double nowSec()
{
  using namespace std::chrono;
  return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

static std::string ipToStr(uint32_t ip)
{
  struct in_addr addr;
  addr.s_addr = htonl(ip);
  char buf[INET_ADDRSTRLEN] = {};
  inet_ntop(AF_INET, &addr, buf, INET_ADDRSTRLEN);
  return std::string(buf);
}

static void pruneTS(std::deque<double>& ts)
{
  double cutoff = nowSec() - FREQ_WINDOW_SEC;
  while (!ts.empty() && ts.front() < cutoff)
    ts.pop_front();
}

static double calcFreq(std::deque<double>& ts)
{
  pruneTS(ts);
  if (ts.size() < 2)
    return 0.0;
  double elapsed = ts.back() - ts.front();
  return (elapsed < 1e-9) ? 0.0 : static_cast<double>(ts.size() - 1) / elapsed;
}

static int openUDPSocket(int port)
{
  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0)
  {
    perror("socket");
    return -1;
  }
  int opt = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
  {
    perror("bind");
    close(fd);
    return -1;
  }
  return fd;
}

struct GCInfo
{
  uint16_t messageBudget[2] = { 0, 0 };
  uint8_t teamNumbers[2] = { 0, 0 };
  std::deque<double> ts;
};
std::map<std::string, GCInfo> g_gcData;

static void handleGC(int fd)
{
  char buf[4096];
  struct sockaddr_in src{};
  socklen_t srcLen = sizeof(src);
  ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&src), &srcLen);
  if (n < 0 || static_cast<size_t>(n) < sizeof(RoboCupGameControlData))
    return;
  const auto* d = reinterpret_cast<const RoboCupGameControlData*>(buf);
  if (strncmp(d->header, GAMECONTROLLER_STRUCT_HEADER, 4) != 0)
    return;
  std::string ip = ipToStr(ntohl(src.sin_addr.s_addr));
  auto& info = g_gcData[ip];
  for (int i = 0; i < 2; i++)
  {
    info.teamNumbers[i] = d->teams[i].teamNumber;
    info.messageBudget[i] = d->teams[i].messageBudget;
  }
  info.ts.push_back(nowSec());
}

struct ProtoInfo
{
  uint32_t robot_id = 0;
  uint32_t team_id = 0;
  size_t minSize = std::numeric_limits<size_t>::max();
  size_t maxSize = 0;
  std::deque<double> ts;
};
std::map<std::string, ProtoInfo> g_protoData;
std::deque<double> g_allProtoTS;

static void handleProto(int fd)
{
  static std::vector<char> buf(65536);
  struct sockaddr_in src{};
  socklen_t srcLen = sizeof(src);
  ssize_t n = recvfrom(fd, buf.data(), buf.size(), 0, reinterpret_cast<sockaddr*>(&src), &srcLen);
  if (n <= 0)
    return;
  hl_communication::GameMsg msg;
  if (!msg.ParseFromArray(buf.data(), static_cast<int>(n)))
    return;
  if (!msg.has_robot_msg())
    return;
  const auto& rm = msg.robot_msg();
  if (!rm.has_robot_id())
    return;
  std::string ip = ipToStr(ntohl(src.sin_addr.s_addr));
  double t = nowSec();
  auto& info = g_protoData[ip];
  if (rm.robot_id().has_robot_id())
    info.robot_id = rm.robot_id().robot_id();
  if (rm.robot_id().has_team_id())
    info.team_id = rm.robot_id().team_id();
  info.minSize = std::min(info.minSize, static_cast<size_t>(n));
  info.maxSize = std::max(info.maxSize, static_cast<size_t>(n));
  info.ts.push_back(t);
  g_allProtoTS.push_back(t);
}

static void display()
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(1);

  out << BOLD << "=== Port 3838 : RoboCupGameControlData ===" << RESET << "\n";
  if (g_gcData.size() > 1)
    out << RED << "  [ERREUR : plusieurs game controllers detectes !]\n" << RESET;
  if (g_gcData.empty())
  {
    out << "  (aucune donnee)\n";
  }
  else
  {
    for (auto& [ip, info] : g_gcData)
    {
      double freq = calcFreq(info.ts);
      out << "  " << std::left << std::setw(16) << ip << "  " << std::setw(6) << freq << " msg/s\n";
      for (int i = 0; i < 2; i++)
        out << "    Equipe#" << static_cast<int>(info.teamNumbers[i]) << "  messageBudget: " << info.messageBudget[i]
            << "\n";
    }
  }
  out << "\n";

  out << BOLD << "=== Port 10011 : Protobuf GameMsg ===" << RESET << "\n";
  using ProtoKey = std::pair<uint32_t, uint32_t>;
  std::map<ProtoKey, std::vector<std::string>> protoPairIPs;
  for (auto& [ip, info] : g_protoData)
    protoPairIPs[{ info.team_id, info.robot_id }].push_back(ip);
  if (g_protoData.empty())
  {
    out << "  (aucune donnee)\n";
  }
  else
  {
    for (auto& [ip, info] : g_protoData)
    {
      size_t minS = (info.minSize == std::numeric_limits<size_t>::max()) ? 0 : info.minSize;
      double freq = calcFreq(info.ts);
      auto key = std::make_pair(info.team_id, info.robot_id);
      bool dup = protoPairIPs[key].size() > 1;
      out << "  " << std::left << std::setw(16) << ip << "  Robot#" << info.robot_id << "  Equipe#" << info.team_id
          << "  taille: " << minS << "-" << info.maxSize << " B"
          << "  " << std::setw(6) << freq << " msg/s";
      if (dup)
        out << "  " << RED << "[ERREUR : Robot#" << info.robot_id << " Equipe#" << info.team_id
            << " envoye par plusieurs IPs !]" << RESET;
      out << "\n";
    }
    pruneTS(g_allProtoTS);
    double globalFreq = 0.0;
    if (g_allProtoTS.size() >= 2)
    {
      double elapsed = g_allProtoTS.back() - g_allProtoTS.front();
      if (elapsed > 1e-9)
        globalFreq = static_cast<double>(g_allProtoTS.size() - 1) / elapsed;
    }
    out << "  - Frequence globale : " << globalFreq << " msg/s\n";
  }
  out << "\n";

  std::cout << "\033[2J\033[H" << out.str() << std::flush;
}

int main()
{
  std::signal(SIGINT, sigHandler);
  std::signal(SIGTERM, sigHandler);

  int fd_gc = openUDPSocket(GAMECONTROLLER_DATA_PORT);
  int fd_proto = openUDPSocket(10011);
  if (fd_gc < 0 || fd_proto < 0)
    return 1;

  struct pollfd fds[2];
  fds[0] = { fd_gc, POLLIN, 0 };
  fds[1] = { fd_proto, POLLIN, 0 };

  double lastDisplay = 0.0;

  while (g_running)
  {
    int ret = poll(fds, 2, POLL_TIMEOUT_MS);
    if (ret > 0)
    {
      if (fds[0].revents & POLLIN)
        handleGC(fd_gc);
      if (fds[1].revents & POLLIN)
        handleProto(fd_proto);
    }
    double t = nowSec();
    if (t - lastDisplay >= REFRESH_SEC)
    {
      display();
      lastDisplay = t;
    }
  }

  close(fd_gc);
  close(fd_proto);
  std::cout << "\033[2J\033[H" << std::flush;
  return 0;
}
