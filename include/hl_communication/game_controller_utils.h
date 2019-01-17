/**
 * This header contains the necessary interface to deal with the non-protobuf
 * game controller messages
 */
#pragma once

#include <hl_communication/game_controller.pb.h>

namespace hl_communication
{

int charsToInt(char const* str, int start, int end);

class Robot{
public:
  Robot();
  ~Robot();

  int getPenalty() const;
  int getSecsTillUnpenalised() const;
  int getYellowCardCount() const;
  int getRedCardCount() const;
  
  /*! \brief Update the robot from a referee box message */
  void updateFromMessage(char const* message, int numRobot);

  void exportToGCRobotMsg(GCRobotMsg * msg) const;

private:
  int penalty;
  int secs_till_unpenalised;
  int yellow_card_count;
  int red_card_count;
};

std::ostream& operator<<(std::ostream& flux, const Robot & r);

class Team
{
public:
  Team();
  ~Team();

  int getTeamNumber() const;
  int getTeamColor() const;
  int getScore() const;
  int getNbRobots() const;
  const Robot & getRobot(int robot) const;

  /*! \brief Update the robot from a referee box message */
  void updateFromMessage(char const* message, int numTeam);

  void exportToGCTeamMsg(GCTeamMsg * msg) const;

private:
  int team_number;
  int team_color;
  int score;
  Robot robots[6];

};

std::ostream& operator<<(std::ostream& flux, const Team & t);

class GameState{
public:
  GameState();
  ~GameState();
  int getStructVersion() const;
  int getGameType() const;
  int getNumPlayer() const;
  int getActualGameState() const;
  int getFirstHalf() const;
  int getKickOffTeam() const;
  int getSecGameState() const;
  int getDropInTeam() const;
  int getDropInTime() const;
  int getEstimatedSecs() const;
  int getSecondarySecs() const;
  int getSecondaryTeam() const;
  int getSecondaryMode() const;
  int getNbTeam() const;
  const Team & getTeam(int teamNumber) const;
  
  /*! \brief Update the robot from a referee box message
   * return true if there has been an update and false if
   * the message was discarded (invalid struct version) */
	bool updateFromMessage(char const* message);

	void show(std::ostream& flux) const;

  void exportToGCMsg(GCMsg * msg) const;
  
private:
  int struct_version;
  int game_type;
  int num_player;
  int actual_game_state;
  int first_half;
  int kick_off_team;
  int sec_game_state;
  int drop_in_team;
  int drop_in_time;
  int estimated_secs;
  int secondary_secs;
  int secondary_team;
  int secondary_mode;
  Team team[2];
};

std::ostream& operator<<(std::ostream& flux, const GameState & gs);

}
