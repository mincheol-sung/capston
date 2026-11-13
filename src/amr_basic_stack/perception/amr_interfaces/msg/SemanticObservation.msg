# Capture-time geometry in header.frame_id. Extents describe observed surfaces,
# not a complete object volume or a grasp pose. Covariance is geometry uncertainty.
std_msgs/Header header
# Index within this capture frame, for tracing; never used as permanent identity.
uint32 detection_index
int32 class_id
string class_name
float32 confidence
geometry_msgs/Point position
geometry_msgs/Point box_center
geometry_msgs/Vector3 extent
geometry_msgs/Point32[] points
float64[9] covariance
float32 valid_depth_ratio
uint8 MASK=0
uint8 BBOX_ROI=1
uint8 support_method
