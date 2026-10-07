# Flame / thermal perception result.
# Published by firefighter_perception once the thermal camera is integrated;
# consumed by the mission FSM. Until then it can be injected by hand for tests.
std_msgs/Header header

uint8 FLAME_FOUND = 0
uint8 FLAME_LOST = 1
uint8 FLAME_EXTINGUISHED = 2
uint8 IN_SUPPRESSION_RANGE = 3

uint8 state

float32 bearing_rad    # flame bearing relative to the robot (+left)
float32 distance_m     # estimated range to the flame base (NaN if unknown)
