uint8 UNKNOWN = 0
uint8 CAR = 1
uint8 TRUCK = 2
uint8 BUS = 3
uint8 TRAILER = 4
uint8 MOTORCYCLE = 5
uint8 BICYCLE = 6
uint8 PEDESTRIAN = 7

std_msgs/Header header
unique_identifier_msgs/UUID object_id
float64 existence_probability
geometry_msgs/Pose pose
geometry_msgs/Pose[] pathes
float64[] path_confidences
geometry_msgs/Vector3 dimension
uint8[] classifications
float64[] classification_confidences
