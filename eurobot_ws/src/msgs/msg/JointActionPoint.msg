# --- JOINT ACTION POINT MESSAGE ---
#   - position (int32 [])  --> servos values (each element of the array corresponds to each servo)
#   - velocity (float32 []) --> motors values (the same as the servos)
#   - activate (bool) --> air pump

string[] servos_names
string[] motors_names

int32[]   position
float32[] velocity
int32     activate