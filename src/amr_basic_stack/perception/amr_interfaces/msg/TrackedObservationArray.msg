# Tracker update epoch; observations retain their own original capture stamps.
# Incremental events, NOT a complete decision for every beam of a scan.
std_msgs/Header header
TrackedObservation[] links
