"""ROS 2 Node for 6PUS inverse and forward kinematics calculations."""

from __future__ import annotations

import math
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray

try:
    from .analytical_6pus import Analytical6PusKinematics
    from .geometry_config import load_geometry_config
    from .kinematics_interface import KinematicsError
except ImportError:
    from analytical_6pus import Analytical6PusKinematics
    from geometry_config import load_geometry_config
    from kinematics_interface import KinematicsError


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """Convert quaternion [qx, qy, qz, qw] to Euler [roll, pitch, yaw] in radians."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class Pus6KinematicsNode(Node):
    """ROS 2 Node computing 6PUS kinematics and enforcing temporary J4 validated software envelope."""

    def __init__(self) -> None:
        super().__init__("pus6_kinematics_node")

        geometry_cfg = load_geometry_config()
        self.declare_parameter("geometry_config_path", str(geometry_cfg.path))
        # Defaults come from YAML; these parameters remain as explicit legacy
        # overrides for launch files that already set them.
        self.declare_parameter("base_radius_m", geometry_cfg.base_radius_m)
        self.declare_parameter("platform_radius_m", geometry_cfg.platform_radius_m)
        self.declare_parameter("strut_length_m", geometry_cfg.strut_length_m)
        self.declare_parameter("nominal_height_m", geometry_cfg.nominal_height_m)
        self.declare_parameter("steps_per_meter", geometry_cfg.steps_per_meter)

        # Temporary J4 Validated Software Envelope bounds (not full physical workspace)
        self.declare_parameter("envelope_max_xy_m", 0.015)             # +/- 15 mm
        self.declare_parameter("envelope_max_z_delta_m", 0.020)        # +/- 20 mm from nominal
        self.declare_parameter("envelope_max_angle_rad", 0.10)         # +/- 0.10 rad (~5.73 deg)
        self.declare_parameter("enforce_software_envelope", True)

        # Legacy backward-compatible parameter aliases
        self.declare_parameter("workspace_max_xy_m", 0.015)
        self.declare_parameter("workspace_max_z_delta_m", 0.020)
        self.declare_parameter("workspace_max_angle_rad", 0.10)
        self.declare_parameter("enforce_workspace_bounds", True)

        base_r = self.get_parameter("base_radius_m").get_parameter_value().double_value
        plat_r = self.get_parameter("platform_radius_m").get_parameter_value().double_value
        strut_l = self.get_parameter("strut_length_m").get_parameter_value().double_value
        self.nom_h = self.get_parameter("nominal_height_m").get_parameter_value().double_value
        spm = self.get_parameter("steps_per_meter").get_parameter_value().double_value
        config_path = self.get_parameter("geometry_config_path").get_parameter_value().string_value

        overrides = {}
        if base_r != geometry_cfg.base_radius_m:
            overrides["base_radius_m"] = base_r
        if plat_r != geometry_cfg.platform_radius_m:
            overrides["platform_radius_m"] = plat_r
        if strut_l != geometry_cfg.strut_length_m:
            overrides["strut_length_m"] = strut_l
        if self.get_parameter("nominal_height_m").get_parameter_value().double_value != geometry_cfg.nominal_height_m:
            overrides["nominal_height_m"] = self.nom_h
        if spm != geometry_cfg.steps_per_meter:
            overrides["steps_per_meter"] = spm

        self.max_xy = self.get_parameter("envelope_max_xy_m").get_parameter_value().double_value
        self.max_z_delta = self.get_parameter("envelope_max_z_delta_m").get_parameter_value().double_value
        self.max_angle = self.get_parameter("envelope_max_angle_rad").get_parameter_value().double_value
        self.enforce_bounds = self.get_parameter("enforce_software_envelope").get_parameter_value().bool_value

        self.kinematics = Analytical6PusKinematics(config_path=config_path, **overrides)
        self.nom_h = self.kinematics.nominal_height

        self.reference_pose = [0.0, 0.0, self.nom_h, 0.0, 0.0, 0.0]

        # Publishers
        self.desired_joints_pub = self.create_publisher(
            Float64MultiArray, "/pus6/desired_joint_positions", 10
        )
        self.desired_deltas_pub = self.create_publisher(
            Float64MultiArray, "/pus6/desired_joint_deltas", 10
        )
        self.desired_delta_steps_pub = self.create_publisher(
            Int32MultiArray, "/pus6/desired_joint_delta_steps", 10
        )

        # Subscriptions
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            "/pus6/target_cartesian_pose",
            self._handle_target_pose_stamped,
            10,
        )
        self.target_array_sub = self.create_subscription(
            Float64MultiArray,
            "/pus6/target_cartesian_array",
            self._handle_target_cartesian_array,
            10,
        )
        self.legacy_pose_sub = self.create_subscription(
            Float64MultiArray,
            "/pus6/target_pose_array",
            self._handle_target_cartesian_array,
            10,
        )

        self.get_logger().info(
            f"Pus6KinematicsNode initialized (spm={spm}, nom_h={self.nom_h:.6f}m, "
            f"temporary_j4_validated_software_envelope=[xy<={self.max_xy*1e3:.1f}mm, "
            f"z_delta<={self.max_z_delta*1e3:.1f}mm, ang<={self.max_angle:.2f}rad])."
        )

    def validate_cartesian_workspace(self, pose: list[float]) -> bool:
        """Check if pose [x, y, z, r, p, y] is within temporary J4 validated software envelope."""
        if not self.enforce_bounds:
            return True

        x, y, z, roll, pitch, yaw = pose
        if abs(x) > self.max_xy or abs(y) > self.max_xy:
            self.get_logger().warn(
                f"Rejected target: planar translation (|X|={abs(x)*1e3:.1f}mm, |Y|={abs(y)*1e3:.1f}mm) "
                f"exceeds temporary J4 software envelope max {self.max_xy*1e3:.1f}mm"
            )
            return False

        z_delta = abs(z - self.nom_h)
        if z_delta > self.max_z_delta:
            self.get_logger().warn(
                f"Rejected target: vertical delta (|Z-z0|={z_delta*1e3:.1f}mm) "
                f"exceeds temporary J4 software envelope max {self.max_z_delta*1e3:.1f}mm"
            )
            return False

        for name, ang in [("Roll", roll), ("Pitch", pitch), ("Yaw", yaw)]:
            if abs(ang) > self.max_angle:
                self.get_logger().warn(
                    f"Rejected target: {name} angle ({abs(ang):.3f}rad) "
                    f"exceeds temporary J4 software envelope max {self.max_angle:.3f}rad"
                )
                return False

        return True

    def _process_and_publish_pose(self, pose: list[float]) -> None:
        """Compute IK and publish joint targets."""
        if not self.validate_cartesian_workspace(pose):
            return

        try:
            joints = self.kinematics.inverse_kinematics(pose)
            deltas = self.kinematics.differential_inverse_kinematics(pose, self.reference_pose)
            delta_steps = self.kinematics.differential_inverse_kinematics_steps(pose, self.reference_pose)

            out_msg = Float64MultiArray()
            out_msg.data = [float(q) for q in joints]
            self.desired_joints_pub.publish(out_msg)

            delta_msg = Float64MultiArray()
            delta_msg.data = [float(dq) for dq in deltas]
            self.desired_deltas_pub.publish(delta_msg)

            step_msg = Int32MultiArray()
            step_msg.data = [int(ds) for ds in delta_steps]
            self.desired_delta_steps_pub.publish(step_msg)

            self.get_logger().debug(f"Published IK delta_steps: {step_msg.data}")
        except KinematicsError as exc:
            self.get_logger().error(f"IK computation failed: {exc}")

    def _handle_target_pose_stamped(self, msg: PoseStamped) -> None:
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        pose = [px, py, pz, roll, pitch, yaw]
        self._process_and_publish_pose(pose)

    def _handle_target_cartesian_array(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 6:
            self.get_logger().error(f"Expected 6-element pose [x,y,z,r,p,y], got {len(msg.data)}")
            return
        self._process_and_publish_pose(list(msg.data))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Pus6KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
