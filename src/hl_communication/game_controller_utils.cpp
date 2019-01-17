#include <hl_communication/game_controller_utils.h>

#include <iostream>

namespace hl_communication
{

static const int nb_chars_by_robot = 4;
static const int nb_chars_by_team = 260 + nb_chars_by_robot * 12;
static const int protocol_version = 12;
static const char * game_state_header = "RGme";


int charsToInt(char const* str, int start, int end){
  int sum = 0;
  int i = 0;
  int mult = 1;
  while (start + i < end){
    if (start + i < end - 1 && str[start + i] < 0)
      sum += mult * (256 + str[start + i]);
    else
      sum += str[start + i] * mult;
    i++;
    mult *= 256;
  }
  return sum;
}

Robot::Robot(){
	penalty = 0;
	secs_till_unpenalised = 0;
}

Robot::~Robot(){
}

int Robot::getPenalty() const{
  return penalty;
}

int Robot::getSecsTillUnpenalised() const{
  return secs_till_unpenalised;
}

int Robot::getYellowCardCount() const{
  return yellow_card_count;
}

int Robot::getRedCardCount() const{
  return red_card_count;
}

/* Use a broadcasted message to update the Robot */
void Robot::updateFromMessage(char const* message, int numRobot){
  int d = nb_chars_by_robot * numRobot;
  penalty = charsToInt(message, d+0, d+1);
  secs_till_unpenalised = charsToInt(message, d+1, d+2);
  yellow_card_count = charsToInt(message, d+2, d+3);
  red_card_count = charsToInt(message, d+3, d+4);
}


void Robot::exportToGCRobotMsg(GCRobotMsg * msg) const {
  msg->Clear();
  msg->set_penalty(getPenalty());
  msg->set_secs_till_unpenalised(getSecsTillUnpenalised());
  msg->set_yellow_card_count(getYellowCardCount());
  msg->set_red_card_count(getRedCardCount());
}

std::ostream& operator<<(std::ostream&flux, const Robot & r){
  flux << "\t\tpenalty : " << r.getPenalty() << std::endl;
  flux << "\t\tsecs_till_unpenalised : " << r.getSecsTillUnpenalised() << std::endl;
  flux << "\t\tyellow_card_count : " << r.getYellowCardCount() << std::endl;
  flux << "\t\tred_card_count : " << r.getRedCardCount() << std::endl;
  return flux;
}

Team::Team(){
  team_number = 0;
  team_color = 0;
  score = 0;
}

Team::~Team(){
}

int Team::getTeamNumber() const{
  return team_number;
}

int Team::getTeamColor() const{
  return team_color;
}

int Team::getScore() const{
  return score;
}

int Team::getNbRobots() const{
  return 6;
}

const Robot & Team::getRobot(int robot) const{
  return robots[robot];
}

/* Use a broadcasted message to update the Robot */
void Team::updateFromMessage(char const* message,int numTeam){
  int d = nb_chars_by_team * numTeam;//offset
  team_number = charsToInt(message, d+0, d+1);
  team_color = charsToInt(message, d+1, d+2);
  score = charsToInt(message, d+2, d+3);
  for (int robot = 0; robot < 2; robot++){
    robots[robot].updateFromMessage(message+d+260+nb_chars_by_robot, robot);
  }
}

void Team::exportToGCTeamMsg(GCTeamMsg * msg) const {
  msg->Clear();
  msg->set_team_number(getTeamNumber());
  msg->set_team_color(getTeamColor());
  msg->set_score(getScore());
  for (int idx = 0; idx < 6; idx++) {
    getRobot(idx).exportToGCRobotMsg(msg->add_robots());
  }
}

std::ostream& operator<<(std::ostream& flux, const Team & t){
  flux << '\t' << "team_number : " << t.getTeamNumber() << std::endl;
  flux << '\t' << "team_color : " << t.getTeamColor() << std::endl;
  flux << '\t' << "score : " << t.getScore() << std::endl;
  for (int robot = 0; robot < 6; robot++){
    flux << '\t' << "robot " << robot << std::endl;
    flux << t.getRobot(robot);
  }
  return flux;
}

GameState::GameState(){
	struct_version = -1;
	num_player = -1;
	actual_game_state = -1;
	first_half = -1;
	kick_off_team = -1;
	sec_game_state = -1;
	drop_in_team = -1;
	drop_in_time = -1;
	estimated_secs = -1;
  secondary_team = -1;
  secondary_mode = -1;
}

GameState::~GameState(){
}

bool GameState::updateFromMessage(char const* message){
  if (strncmp(message, game_state_header, 4) != 0) {
    return false;
  }
  struct_version = charsToInt(message, 4, 6);

  if (struct_version != protocol_version) {
    std::cerr << "Game controller bad version " << struct_version << std::endl;
    return false;
  }

  num_player = charsToInt(message, 7, 8);
  game_type = charsToInt(message, 8, 9);
  actual_game_state = charsToInt(message, 9, 10);
  first_half = charsToInt(message, 10, 11);
  kick_off_team = charsToInt(message, 11, 12);
  sec_game_state = charsToInt(message, 12, 13);
  secondary_team = charsToInt(message, 13, 14);
  secondary_mode = charsToInt(message, 14, 15);

  drop_in_team = charsToInt(message, 17, 18);
  drop_in_time = charsToInt(message, 18, 20);
  estimated_secs = charsToInt(message, 20, 22);
  secondary_secs = charsToInt(message, 22, 24);


  for (int i = 0; i < 2; i++)
    team[i].updateFromMessage(message+24, i);
  return true;
}

int GameState::getStructVersion() const{
  return struct_version;
}

int GameState::getGameType() const{
  return game_type;
}

int GameState::getNumPlayer() const{
  return num_player;
}

int GameState::getActualGameState() const{
  return actual_game_state;
}

int GameState::getFirstHalf() const{
  return first_half;
}

int GameState::getKickOffTeam() const{
  return kick_off_team;
}

int GameState::getSecGameState() const{
  return sec_game_state;
}

int GameState::getSecondaryTeam() const{
  return secondary_team;
}

int GameState::getSecondaryMode() const{
  return secondary_mode;
}

int GameState::getDropInTeam() const{
  return drop_in_team;
}

int GameState::getDropInTime() const{
  return drop_in_time;
}

int GameState::getEstimatedSecs() const{
  return estimated_secs;
}
  
int GameState::getSecondarySecs() const{
  return secondary_secs;
}

int GameState::getNbTeam() const{
  return 2;
}

const Team & GameState::getTeam(int teamNumber) const{
  return team[teamNumber];
}

std::ostream& operator<<(std::ostream& flux, const GameState & gs){
	flux << "struct_version : " << gs.getStructVersion() << std::endl;
	flux << "num_player : " << gs.getNumPlayer() << std::endl;
	flux << "gameState : " << gs.getActualGameState() << std::endl;
	flux << "first_half : " << gs.getFirstHalf() << std::endl;
	flux << "kick_off_team : " << gs.getKickOffTeam() << std::endl;
	flux << "sec_game_state : " << gs.getSecGameState() << std::endl;
	flux << "sec_game_mode : " << gs.getSecondaryMode() << std::endl;
	flux << "sec_game_team : " << gs.getSecondaryTeam() << std::endl;
	flux << "secondary_secs : " << gs.getSecondarySecs() << std::endl;
	flux << "drop_in_team : " << gs.getDropInTeam() << std::endl;
	flux << "drop_in_time : " << gs.getDropInTime() << std::endl;
	flux << "estimated_secs : " << gs.getEstimatedSecs() << std::endl;
  for (int i = 0; i < 2; i++){
    flux << "Team " << i << std::endl;
    flux << gs.getTeam(i);
  }
	return flux;
}

void GameState::exportToGCMsg(GCMsg * msg) const {
  msg->Clear();
  msg->set_struct_version(getStructVersion());
  msg->set_game_type(getGameType());
  msg->set_num_player(getNumPlayer());
  msg->set_first_half(getFirstHalf());
  msg->set_kick_off_team(getKickOffTeam());
  msg->set_sec_game_state(getSecGameState());
  msg->set_drop_in_team(getDropInTeam());
  msg->set_drop_in_time(getDropInTime());
  msg->set_estimated_secs(getEstimatedSecs());
  msg->set_secondary_secs(getSecondarySecs());
  msg->set_secondary_mode(getSecondaryMode());
  for (int team = 0; team < 2; team++) {
    getTeam(team).exportToGCTeamMsg(msg->add_teams());
  }
}

}
