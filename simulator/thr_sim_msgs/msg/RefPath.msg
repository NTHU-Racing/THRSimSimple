# Reference path for the driver, in two segments:
#   nodes[0 .. finish_index]   segment to drive
#   nodes[finish_index]        finish (transition) node: stop the vehicle here
#   nodes[finish_index .. end] after-finish (run-out) segment
# An empty `nodes` list means "no task": stop the vehicle.
std_msgs/Header header
geometry_msgs/Point[] nodes
uint32 finish_index
