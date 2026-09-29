#!/usr/bin/env python3

import math
from enum import Enum, auto
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose


class State(Enum):
    WAIT_FOR_DATA  = auto()
    SELECT_GOAL    = auto()
    SEND_GOAL      = auto()
    WAIT_FOR_RESULT = auto()


class MissionManager(Node):

    def __init__(self) -> None:
        super().__init__("mission_manager")

        self.declare_parameter("goal_x",           rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("goal_y",           rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("step_size",        0.25)
        self.declare_parameter("backoff_distance", 1.0)
        self.declare_parameter("cost_threshold",   80)
        self.declare_parameter("replan_rate",      2.0)
        self.declare_parameter("odom_topic",       "/odometry/filtered")

        self._goal_x:           float = self.get_parameter("goal_x").value
        self._goal_y:           float = self.get_parameter("goal_y").value
        self._step_size:        float = self.get_parameter("step_size").value
        self._backoff_distance: float = self.get_parameter("backoff_distance").value
        self._cost_threshold:   int   = int(self.get_parameter("cost_threshold").value)
        self._replan_rate:      float = self.get_parameter("replan_rate").value
        odom_topic:             str   = self.get_parameter("odom_topic").value

        self.get_logger().info(
            f"Parameters — goal=({self._goal_x:.3f}, {self._goal_y:.3f})  "
            f"step={self._step_size}m  backoff={self._backoff_distance}m  "
            f"cost_threshold={self._cost_threshold}  "
            f"replan_rate={self._replan_rate}Hz  odom={odom_topic}"
        )

        self._state: State = State.WAIT_FOR_DATA

        self._map:      Optional[OccupancyGrid] = None
        self._costmap:  Optional[OccupancyGrid] = None
        self._odometry: Optional[Odometry]      = None

        self._goal_handle = None
        self._goal_in_flight: bool = False

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            OccupancyGrid,
            "/map",
            self._map_callback,
            latched_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._costmap_callback,
            latched_qos,
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            sensor_qos,
        )

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        period = 1.0 / self._replan_rate
        self.create_timer(period, self._control_loop)

        self.get_logger().info("MissionManager initialised — waiting for data.")

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _costmap_callback(self, msg: OccupancyGrid) -> None:
        self._costmap = msg

    def _odom_callback(self, msg: Odometry) -> None:
        self._odometry = msg

    def _control_loop(self) -> None:

        if self._state == State.WAIT_FOR_DATA:
            missing = self._missing_data_report()
            if missing:
                self.get_logger().info(
                    f"Waiting for data: {', '.join(missing)}",
                    throttle_duration_sec=5.0,
                )
                return

            self.get_logger().info("All data available — Mission Started.")
            self._log_pose_and_goal()
            self._state = State.SELECT_GOAL

        elif self._state == State.SELECT_GOAL:
            robot_x, robot_y = self._current_pose()

            if self._at_final_goal(robot_x, robot_y):
                self.get_logger().info(
                    "Final goal reached!  Mission complete.  Node idling."
                )
                self._state = State.WAIT_FOR_DATA
                return

            intermediate = self.select_intermediate_goal(
                robot_x, robot_y,
                self._goal_x, self._goal_y,
            )

            if intermediate is None:
                self.get_logger().warn(
                    "No valid intermediate goal found along the ray.  "
                    "Will retry next cycle."
                )
                return

            ix, iy = intermediate
            self.get_logger().info(
                f"Intermediate goal selected: ({ix:.3f}, {iy:.3f})  "
                f"[distance to final: "
                f"{self.distance(ix, iy, self._goal_x, self._goal_y):.2f}m]"
            )
            self._pending_goal = intermediate
            self._state = State.SEND_GOAL

        elif self._state == State.SEND_GOAL:
            ix, iy = self._pending_goal
            self._send_goal(ix, iy)
            self._state = State.WAIT_FOR_RESULT

        elif self._state == State.WAIT_FOR_RESULT:
            pass

    def select_intermediate_goal(
        self,
        robot_x: float, robot_y: float,
        goal_x:  float, goal_y:  float,
    ) -> Optional[Tuple[float, float]]:
        dx, dy = self.normalize_vector(goal_x - robot_x, goal_y - robot_y)
        total_dist = self.distance(robot_x, robot_y, goal_x, goal_y)

        best_x: Optional[float] = None
        best_y: Optional[float] = None

        step = self._step_size
        travelled = step

        while travelled <= total_dist:
            sx = robot_x + dx * travelled
            sy = robot_y + dy * travelled

            if not self.is_inside_map(sx, sy, self._map):
                self.get_logger().debug(
                    f"  Sample ({sx:.2f},{sy:.2f}) outside map — stopping ray."
                )
                break

            occ = self.occupancy_value(sx, sy, self._map)

            if occ == -1:
                self.get_logger().debug(
                    f"  Sample ({sx:.2f},{sy:.2f}) is unknown — stopping ray."
                )
                break

            if occ >= 50:
                self.get_logger().debug(
                    f"  Sample ({sx:.2f},{sy:.2f}) is occupied (occ={occ}) — "
                    "stopping ray."
                )
                break

            cost = self.costmap_value(sx, sy, self._costmap)

            if cost < 0 or cost > self._cost_threshold:
                self.get_logger().debug(
                    f"  Sample ({sx:.2f},{sy:.2f}) cost={cost} exceeds "
                    f"threshold {self._cost_threshold} — stopping ray."
                )
                break

            best_x, best_y = sx, sy
            travelled += step

        if best_x is None:
            return None

        best_dist = self.distance(robot_x, robot_y, best_x, best_y)
        backed_dist = max(0.0, best_dist - self._backoff_distance)

        final_x = robot_x + dx * backed_dist
        final_y = robot_y + dy * backed_dist

        return (final_x, final_y)

    def _send_goal(self, x: float, y: float) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "NavigateToPose action server not available after 5 s.  "
                "Reverting to SELECT_GOAL."
            )
            self._state = State.SELECT_GOAL
            return

        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"Sending goal to Nav2: ({x:.3f}, {y:.3f})")

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_future.add_done_callback(self.goal_response_callback)
        self._goal_in_flight = True

    def goal_response_callback(self, future) -> None:
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            self.get_logger().warn(
                "Goal was REJECTED by Nav2.  Returning to SELECT_GOAL."
            )
            self._goal_in_flight = False
            self._state = State.SELECT_GOAL
            return

        self.get_logger().info("Goal ACCEPTED by Nav2.")
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future) -> None:
        result   = future.result()
        status   = result.status
        self._goal_in_flight = False
        self._goal_handle    = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                "Goal REACHED.  Selecting next intermediate goal."
            )
            self._state = State.SELECT_GOAL

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                "Goal CANCELLED.  Returning to SELECT_GOAL."
            )
            self._state = State.SELECT_GOAL

        else:
            self.get_logger().warn(
                f"Goal FAILED (status={status}).  Returning to SELECT_GOAL."
            )
            self._state = State.SELECT_GOAL

    def feedback_callback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().debug(
            f"  Nav2 feedback — distance_remaining: {dist:.2f} m",
            throttle_duration_sec=2.0,
        )

    @staticmethod
    def normalize_vector(dx: float, dy: float) -> Tuple[float, float]:
        mag = math.hypot(dx, dy)
        if mag < 1e-9:
            return (0.0, 0.0)
        return (dx / mag, dy / mag)

    @staticmethod
    def distance(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    @staticmethod
    def world_to_map(
        wx: float, wy: float, grid: OccupancyGrid
    ) -> Tuple[int, int]:
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        res = grid.info.resolution

        col = int((wx - ox) / res)
        row = int((wy - oy) / res)
        return (col, row)

    @staticmethod
    def map_to_world(
        col: int, row: int, grid: OccupancyGrid
    ) -> Tuple[float, float]:
        ox  = grid.info.origin.position.x
        oy  = grid.info.origin.position.y
        res = grid.info.resolution

        wx = ox + (col + 0.5) * res
        wy = oy + (row + 0.5) * res
        return (wx, wy)

    @staticmethod
    def is_inside_map(wx: float, wy: float, grid: OccupancyGrid) -> bool:
        ox  = grid.info.origin.position.x
        oy  = grid.info.origin.position.y
        res = grid.info.resolution
        w   = grid.info.width
        h   = grid.info.height

        col = int((wx - ox) / res)
        row = int((wy - oy) / res)
        return 0 <= col < w and 0 <= row < h

    @staticmethod
    def occupancy_value(wx: float, wy: float, grid: OccupancyGrid) -> int:
        ox  = grid.info.origin.position.x
        oy  = grid.info.origin.position.y
        res = grid.info.resolution
        w   = grid.info.width

        col = int((wx - ox) / res)
        row = int((wy - oy) / res)

        if not (0 <= col < grid.info.width and 0 <= row < grid.info.height):
            return -1

        return grid.data[row * w + col]

    @staticmethod
    def costmap_value(wx: float, wy: float, grid: OccupancyGrid) -> int:
        ox  = grid.info.origin.position.x
        oy  = grid.info.origin.position.y
        res = grid.info.resolution
        w   = grid.info.width

        col = int((wx - ox) / res)
        row = int((wy - oy) / res)

        if not (0 <= col < grid.info.width and 0 <= row < grid.info.height):
            return -1

        return grid.data[row * w + col]

    def _current_pose(self) -> Tuple[float, float]:
        pos = self._odometry.pose.pose.position
        return (pos.x, pos.y)

    def _at_final_goal(self, robot_x: float, robot_y: float) -> bool:
        return (
            self.distance(robot_x, robot_y, self._goal_x, self._goal_y)
            < self._step_size
        )

    def _missing_data_report(self) -> list:
        missing = []
        if self._map      is None: missing.append("map")
        if self._costmap  is None: missing.append("costmap")
        if self._odometry is None: missing.append("odometry")
        return missing

    def _log_pose_and_goal(self) -> None:
        rx, ry = self._current_pose()
        self.get_logger().info(
            f"Current Pose: ({rx:.3f}, {ry:.3f})  |  "
            f"Final Goal:   ({self._goal_x:.3f}, {self._goal_y:.3f})  |  "
            f"Distance:     {self.distance(rx, ry, self._goal_x, self._goal_y):.2f} m"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()