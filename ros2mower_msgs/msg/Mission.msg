# Actual mission of robot


uint8 MISSION_IDLE = 0
uint8 MISSION_AREA_RECORDING = 10
uint8 MISSION_SINGLE_MOW = 20
uint8 MISSION_MOW = 21
uint8 MISSION_DOCKING = 30
uint8 MISSION_STATION = 80
uint8 MISSION_ERROR = 99

std_msgs/Header header
uint8 mission
std_msgs/String mission_data
