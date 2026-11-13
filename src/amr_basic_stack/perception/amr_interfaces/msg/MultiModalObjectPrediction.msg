uint32 track_id
int32 class_id
string class_name
float32 radius
uint8 motion_state

# Observation quality is separate from CV/CT model probabilities.
uint8 lifecycle_state
float64 measurement_age
float64 semantic_age
int8 last_measurement_source
float32 class_confidence

# Relative to MultiModalObjectPredictionArray.header.stamp.
# The first entry is 0.0 and contains the current tracker posterior. Remaining
# entries are future predictions. This lets consumers represent STATIC/UNKNOWN
# objects at the measured current position instead of the first future point.
float32[] time_offsets
MotionModePrediction[] modes
