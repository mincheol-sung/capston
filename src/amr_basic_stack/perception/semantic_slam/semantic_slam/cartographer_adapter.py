"""Nonblocking, bounded ROS service polling for optimized graph snapshots."""

import time

from cartographer_ros_msgs.srv import GetTrajectoryStates
from cartographer_ros_msgs.srv import TrajectoryQuery

from semantic_slam.pose_graph import NodePose
from semantic_slam.pose_graph import PoseGraph


class CartographerAdapter:
    """Commit all queried trajectories together; fail closed on partial replies."""

    def __init__(self, node, memory, parameters):
        self.node, self.memory, self.p = node, memory, parameters
        self.graph = PoseGraph(parameters['cartographer_graph_id'],
                               parameters['graph_max_interpolation_gap'],
                               parameters['graph_max_nodes'])
        self.states = node.create_client(
            GetTrajectoryStates, parameters['cartographer_states_service'])
        self.query = node.create_client(TrajectoryQuery, parameters['cartographer_query_service'])
        self.future, self.client, self.phase = None, None, ''
        self.last_poll, self.last_success = -float('inf'), -float('inf')
        self.deadline, self.valid = 0.0, False
        self.queue, self.staged, self.current = [], {}, None
        self.lost = set()
        self.error = ''
        if (parameters['graph_poll_period'] <= 0 or parameters['graph_service_timeout'] <= 0
                or parameters['graph_max_trajectories'] < 1):
            raise ValueError('Invalid pose graph polling limits')

    @property
    def ready(self):
        return self.valid and time.monotonic() - self.last_success <= self.p['graph_freshness']

    def invalidate(self):
        """Ignore in-flight replies around trajectory reset or clock discontinuity."""
        if self.future:
            self.client.remove_pending_request(self.future)
            self.future.cancel()
        self.future, self.valid = None, False
        self.graph.active = None
        self.last_poll = -float('inf')

    def _fail(self, message):
        self.valid = False
        if self.future:
            self.client.remove_pending_request(self.future)
            self.future.cancel()
        self.future = None
        if message != self.error:
            self.node.get_logger().warning(f'Pose graph unavailable: {message}')
            self.error = message

    def _request(self, client, request, phase):
        self.client, self.phase = client, phase
        self.future = client.call_async(request)
        self.deadline = time.monotonic() + self.p['graph_service_timeout']

    def tick(self):
        """Perform at most one state transition per timer callback."""
        now = time.monotonic()
        if self.future:
            if not self.future.done():
                if now > self.deadline:
                    self._fail('service timeout')
                return
            future, phase = self.future, self.phase
            self.future = None
            try:
                response = future.result()
                if response is None or response.status.code != 0:
                    raise ValueError('service returned an error')
                if phase == 'states':
                    self._states(response)
                else:
                    self._trajectory(response)
            except (ValueError, RuntimeError, KeyError, TypeError) as exc:
                self._fail(str(exc))
            return
        if now - self.last_poll < self.p['graph_poll_period']:
            return
        self.last_poll = now
        if not self.states.service_is_ready() or not self.query.service_is_ready():
            self._fail('Cartographer services are not ready')
            return
        self._request(self.states, GetTrajectoryStates.Request(), 'states')

    def _states(self, response):
        states = response.trajectory_states
        if len(states.trajectory_id) != len(states.trajectory_state):
            raise ValueError('Malformed trajectory state arrays')
        pairs = dict(zip(states.trajectory_id, states.trajectory_state))
        active = [key for key, state in pairs.items() if state == 0]
        requested = self.p['cartographer_trajectory_id']
        if requested >= 0:
            active = [requested] if requested in active else []
        if len(active) > 1:
            raise ValueError('Exactly one active trajectory is required')
        # With no ACTIVE trajectory (e.g. after finish_trajectory), new observations
        # cannot be bound, but the final optimization must still reach saved anchors.
        self.active = active[0] if active else None
        if self.graph.active != self.active:
            self.valid = False
        latest = {}
        for record in self.memory.objects.values():
            for anchor in record.anchors:
                key = anchor['trajectory_id']
                latest[key] = max(latest.get(key, -float('inf')), anchor['stamp'])
        # DELETED or unknown trajectories can never support an anchor again; they
        # are not queried and do not count against the query capacity.
        self.lost = {key for key in latest if pairs.get(key, 3) == 3}
        queryable = (set(latest) - self.lost) | set(active)
        if not queryable and not self.lost:
            raise ValueError('No active trajectory and no anchored objects to reproject')
        skipped = set()
        capacity = self.p['graph_max_trajectories']
        if len(queryable) > capacity:
            # Old evidence must not block the current trajectory: keep the active
            # one and the most recently observed; the rest become unsupported.
            others = sorted(queryable - set(active), key=lambda k: latest[k], reverse=True)
            kept = set(active) | set(others[:capacity - len(active)])
            skipped, queryable = queryable - kept, kept
            self.node.get_logger().warning(
                f'Referenced trajectory capacity exceeded; not querying {sorted(skipped)}',
                throttle_duration_sec=30.0)
        self.staged = {key: [] for key in self.lost | skipped}
        self.queue = sorted(queryable)
        self._next()

    def _trajectory(self, response):
        if len(response.trajectory) > self.p['graph_max_nodes']:
            raise ValueError('Trajectory node capacity exceeded')
        nodes = []
        for stamped in response.trajectory:
            if stamped.header.frame_id != self.p['map_frame']:
                raise ValueError('Trajectory map frame mismatch')
            p, q = stamped.pose.position, stamped.pose.orientation
            stamp = stamped.header.stamp.sec + stamped.header.stamp.nanosec * 1e-9
            nodes.append(NodePose(stamp, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
        self.staged[self.current] = nodes
        self._next()

    def _next(self):
        if self.queue:
            self.current = self.queue.pop(0)
            self._request(self.query, TrajectoryQuery.Request(trajectory_id=self.current), 'query')
            return
        graph = PoseGraph(self.graph.graph_id, self.graph.max_gap, self.graph.max_nodes)
        graph.continue_from(self.graph)
        for key, nodes in self.staged.items():
            graph.replace(key, nodes)
        graph.active = self.active
        if self.active is not None:
            if len(graph.nodes.get(self.active, {})) < 2:
                raise ValueError('Active trajectory has fewer than two nodes')
            now_ros = self.node.get_clock().now().nanoseconds / 1e9
            latest = max(graph.nodes[self.active])
            if not -0.1 <= now_ros - latest <= self.p['graph_node_timeout']:
                raise ValueError('Active trajectory nodes are stale')
        self.memory.prune_anchors(self.lost)
        self.memory.reproject(graph)
        self.graph = graph
        self.last_success = time.monotonic()
        if self.active is None:
            # Reprojection-only: keep polling, but refuse new bindings and missions.
            self.valid = False
            message = 'No active trajectory; reprojecting finished trajectories only'
            if message != self.error:
                self.node.get_logger().warning(f'Pose graph unavailable: {message}')
                self.error = message
        else:
            self.valid, self.error = True, ''
