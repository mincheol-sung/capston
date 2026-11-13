"""Export the actual tracker measurement assignment without geometric reassociation."""

from copy import deepcopy

from amr_interfaces.msg import TrackedObservation, TrackedObservationArray


def observation_links(tracks, matches, detections, header):
    """Use only surviving/published tracks and measurements consumed this update."""
    out = TrackedObservationArray()
    out.header = deepcopy(header)
    for track in tracks:
        index = matches.get(track.id)
        if index is None:
            continue
        for observation in getattr(detections[index], 'observations', []):
            # Camera-only and empty scan records cannot identify LiDAR beams.
            if not observation.beam_indices:
                continue
            link = TrackedObservation()
            link.track_id = int(track.id)
            link.lifecycle_state = int(track.lifecycle)
            link.motion_state = int(track.motion)
            link.class_name = track.class_name
            link.semantic_age = (max(0., track.stamp - track.last_semantic_update)
                                 if track.last_semantic_update is not None else -1.)
            link.observation = deepcopy(observation)
            out.links.append(link)
    return out
