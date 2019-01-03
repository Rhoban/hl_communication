#include <vector>

namespace rhl {

/**
 * 2 Referential are considered, self and field, but only SI units are used
 * - Field:
 *   - Center is 0,0
 *   - X-axis goes from center to opposite goal
 *   - Y-axis goes from center toward left side of the field while facing opposite goal
 * - Self:
 *   - Center is the robot position
 *   - X-axis goes in front of the robot
 *   - Y-axis goes to the left of the robot
 */

/**
 * Estimation of the position of an object with uncertainty
 */
class PositionEstimation {
  /**
   * Position x [m]
   */
  float x;
  /**
   * Position y [m]
   */
  float y;

  /**
   * - If empty: not informed
   * - If size 2: ( std_dev_x , std_dev_y )
   * - If size 3: ( covar_x , covar_xy, covar_y)
   */
  std::vector<float> uncertainty;
};

/**
 * Estimation of the pose of an object (position + orientation)
 */
class PoseEstimation {
  /**
   * Position x [m]
   */
  float x;
  /**
   * Position y [m]
   */
  float y;
  /**
   * Direction of the object [rad]
   */
  float dir;

  /**
   * Flag used in order to indicate if the provided 'dir' can be used
   */
  bool useDir;

  /**
   * Note: Use of std_dev and covar for direction are not satisfying here (see Von Mises)
   * - If empty: not informed
   * - If size 3: ( std_dev_x , std_dev_y, std_dev_dir )
   * - If size 6: ( covar_x , covar_xy, covar_xdir, covar_y, covar_ydir, covar_dir)
   */
  std::vector<float> uncertainty;
};

enum TeamID {
  UNKNOWN = 0,
  ALLY = 1,
  OPPONENT = 2
};

class RobotEstimation {
  /**
   * Team identifier
   */
  TeamID teamId;

  /**
   * Identifier of the robot:
   * - 0: unknown id
   * - 1+: Identifier of the robot
   */
  unsigned char robotId;
  
  /**
   * Pose of the robots in self referential
   */
  PoseEstimation robotInSelf;
  
};

/**
 * Perception of the world state by the robot
 */
class PerceptionMessage {
  PositionEstimation ballInSelf;
  /**
   * Estimation of the pose of the robot in the field referential allowing
   * multiple candidates.
   * - First element represents the probability of the pose
   * - Second element describes the pose
   */
  std::vector<std::pair<float,PoseEstimation>> selfInField;

  /**
   * Note: might be inconsistent with 'selfInField'
   */
  PositionEstimation oppGoalInSelf;
  /**
   * Indicates whether oppGoalInSelf has been filled or not
   */
  bool useOppGoalInSelf;

  /**
   * Belief regarding positions of other robots in self referential
   * - First element represents the probability that the information is accurate
   * - Second element describes the robot
   */
  std::vector<std::pair<float,RobotEstimation>> robots;

  /**
   * Up to teams
   */
  std::string freeField;
};

enum Action {
  UNDEFINED        = 0,// None of the action matches current action
  POSITIONING      = 1,// Robot is repositionning (but do not plan to kick immediately)
  GOING_TO_KICK    = 2,// Robot is moving toward the ball in order to kick
  KICKING          = 3,// Robot is currently perforimg the kick motion
  GOING_TO_DRIBBLE = 4,// Robot is moving toward the ball and plans to dribble
  DRIBBLING        = 5,// Robot is actively dribbling
  WAITING          = 6 // Robot is waiting for something to happen  
};

/**
 * Describes the intention of the robot
 */
class IntentionMessage {
  /**
   * Destination of the robot in field referential
   */
  PoseEstimation targetPoseInField;

  /**
   * List of waypoints (when avoiding an obstacle for example), first element of
   * the vector is the next waypoint
   */
  std::vector<PoseEstimation> waypointsInField;

  /**
   * Current action planned by the robot
   */
  Action actionPlanned;

  /**
   * Estimated result for the ball position after next kick, when ball has
   * stopped
   */
  PositionEstimation kickTargetInField;

  /**
   * Up to teams
   */
  std::string freeField;
};

/**
 * Global task of the robot
 */
enum Role {
  UNSPECIFIED = 0, // Unknown role
  GOALIE      = 1, // The robot is a goalie
  DEFENDER    = 2, // Robot is only defending its own goal
  OFFENDER    = 3  // Robot is actively trying to score goals
};

/**
 * Current status of the robot
 */
enum Status {
  UNSPECIFIED      = 0, // Unknown status
  OKAY             = 1, // Robot is playing without any disfunction
  FALLING          = 2, // Robot is currently falling
  FALLEN           = 3, // Robot is fallen on the ground and is not attempting to stand up
  STANDING_UP      = 4, // Robot is actively trying to stand up
  FREEZE           = 5, // Robot is freezing all motion in order to fulfill the rules
  HANDLED          = 6, // Robot is handled (carried by the handler)
  HARDWARE_FAILURE = 7  // Robot has a major hardware failure
};

/**
 * Should we include the possiblity of having a captain in the protocol?
 */
class TeamPlayMessage {
  Role role;

  Status status;
  
  /**
   * Up to teams
   */
  std::string freeField;
};

class CapabilitiesMessage {
  // Maximal walking speed of the robot [m/s]
  float maxWalkingSpeed;
  // Average kicking distance for robot kicks [m]
  float avgKickDistance;

  /**
   * Up to teams
   */
  std::string freeField;
};

}
