"""A delayed TF reaches a pending detection without a blocking lookup."""
from types import SimpleNamespace

from amr_interfaces.msg import DetectedObjectArray

from geometry_msgs.msg import TransformStamped

from object_tracker.core.pending import PendingMeasurements
from object_tracker.object_tracker_node import ObjectTrackerNode

from tf2_ros import Buffer


def test_actual_tf_buffer_can_be_filled_between_nonblocking_retries():
    """A second retry can use a transform inserted after the first one."""
    buffer = Buffer()
    node = SimpleNamespace(track_frame='odom', tf_buffer=buffer)
    message = DetectedObjectArray()
    message.header.frame_id = 'base_link'
    message.header.stamp.sec = 2
    queue = PendingMeasurements()
    queue.add(2000000000, message, 0.)
    completed = []

    def lookup(m):
        return ObjectTrackerNode._lookup(node, m.header.frame_id, m.header.stamp)

    def consume(m, tf):
        completed.append(tf)

    queue.drain(.01, lookup, consume)
    assert completed == []
    transform = TransformStamped()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'base_link'
    transform.header.stamp.sec = 2
    transform.transform.rotation.w = 1.
    transform.transform.translation.x = 3.
    buffer.set_transform(transform, 'test')
    queue.drain(.02, lookup, consume)
    assert len(completed) == 1
    assert completed[0][0].tolist() == [3., 0.]
