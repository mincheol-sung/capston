# Internal object_tracker -> motion_predictor posterior contract.
uint8 LIFECYCLE_TENTATIVE = 0
uint8 LIFECYCLE_CONFIRMED = 1
uint8 LIFECYCLE_LOST = 2
uint8 LIFECYCLE_DELETED = 3

uint32 track_id
int32 class_id
string class_name
float32 confidence
float32 radius
uint8 lifecycle_state
uint8 motion_state

# Ages at array header.stamp. semantic_age=-1 means never class-confirmed.
float64 measurement_age
float64 semantic_age
# 0 camera, 1 lidar, 2 fused, -1 unknown.
int8 last_measurement_source

# Mean mode dwell times.  Consumers derive Pi(dt)=expm(G*dt).
float64 mean_sojourn_cv
float64 mean_sojourn_ct
float64 mean_sojourn_static
# Conditional destination fractions on leaving each source mode.
float64 static_to_cv
float64 cv_to_static
float64 ct_to_static
ImmModeState[] modes
