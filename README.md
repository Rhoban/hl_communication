# HL COMMUNICATION

This project describes the communication protocol to be used for humanoid
league, it is based upon Protobuf.

## Referentials
2 Referentials are considered for the messages, self and field. Only SI units
are used.

- Field:
  - Center is 0,0
  - X-axis goes from center to opposite goal
  - Y-axis goes from center toward left side of the field while facing opposite goal
- Self:
  - Center is the robot position
  - X-axis goes in front of the robot
  - Y-axis goes to the left of the robot
