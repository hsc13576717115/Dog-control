# Evaluation only. Integrated at each physics step, never fed to the controller.
std_msgs/Header header
uint64 physics_sequence
bool[4] contacts
float64[4] cumulative_slip_m
float64[4] cumulative_contact_s
