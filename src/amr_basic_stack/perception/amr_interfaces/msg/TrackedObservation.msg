# A measurement actually consumed in this tracker update, never a prediction.
# State/class constants match ImmTrackedObject and TrackedObject.
uint32 track_id
uint8 lifecycle_state
uint8 motion_state
string class_name
float64 semantic_age
# Original sensor capture identity and beam membership, unchanged by fusion.
ObservationProvenance observation
