# Semantic Object Detector 출력: Position/Class/Confidence
std_msgs/Header            header
int32                      class_id
string                     class_name
float32                    confidence
vision_msgs/BoundingBox2D  bbox
geometry_msgs/Point        position
float32                    distance

# 관측 출처.
#   0 = CAMERA  /detections       (rgbd_detection_node, 카메라 단독)
#   1 = LIDAR   /lidar_clusters   (lidar_cluster_node, 라이다 단독)
#   2 = FUSED   /detections_fused (camera_lidar_fusion_node, 두 관측 융합)
uint8                      source

# header.frame_id 평면(xy) 기준 2x2 측정 공분산, row-major, 단위 m^2.
#   [xx, xy, yx, yy]
# 전부 0 이면 "미기입" 이다. camera_lidar_fusion_node 가 R(d) 모델로 채운다.
# C2 실측(2026-09-04) 근거는 stack_master/scripts/EXPERIMENT_C2.md 참고.
float64[4]                 position_covariance

# header is the coordinate reference epoch, shared with the enclosing array.
# Sensor capture times live only in these immutable provenance records.
ObservationProvenance[]    observations
# True only when every LiDAR member has verified timing and was deskewed.
bool                       ego_motion_compensated
