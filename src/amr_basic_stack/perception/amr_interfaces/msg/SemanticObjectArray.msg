std_msgs/Header header
string map_id
# Increments only when existing observations are reprojected or lose support.
# Routine optimization also increments it; consumers judge their own targets.
uint64 revision
# False while the configured pose graph is unavailable/incomplete.
bool pose_graph_ready
SemanticObject[] objects
SemanticRelation[] relations
