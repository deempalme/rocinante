Header header

int32 id

# shape relative to object's center position
geometry_msgs/Polygon footprint

float32 height

# classification
#  0 : unknown
#  1 : pedestrian
#  2 : cycle
#  3 : vehicle
int8    classification
float32 classification_confidence

# dynamics/tracking
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance velocity
geometry_msgs/AccelWithCovariance acceleration

int32 age
int32 total_age

geometry_msgs/Point[] trajectory
float32 trajectory_width

# object's name
string name
