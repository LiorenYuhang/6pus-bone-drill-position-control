"""pus6_kinematics: Kinematics calculation package for 6PUS parallel manipulator."""

from .analytical_6pus import Analytical6PusKinematics
from .geometry_config import GeometryConfig, default_geometry_config_path, load_geometry_config
from .kinematics_interface import (
    ForwardKinematicsResult,
    KinematicsError,
    KinematicsInterface,
    SingularityError,
    WorkspaceLimitError,
)
from .state_synchronizer import (
    FixedLagJointSynchronizer,
    SyncConfigurationResult,
    SyncStatus,
)


def __getattr__(name: str):
    """Load ROS FK integration only when it is explicitly requested.

    The analytical/J3 helpers are also used by standalone scripts before a ROS
    workspace is sourced. Importing the FK node here would make those helpers
    require rclpy and generated ``pus6_msgs`` at package-import time.
    """
    if name == "Pus6FkNode":
        from .fk_node import Pus6FkNode

        return Pus6FkNode
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = [
    "KinematicsInterface",
    "KinematicsError",
    "SingularityError",
    "WorkspaceLimitError",
    "Analytical6PusKinematics",
    "GeometryConfig",
    "default_geometry_config_path",
    "load_geometry_config",
    "ForwardKinematicsResult",
    "FixedLagJointSynchronizer",
    "SyncConfigurationResult",
    "SyncStatus",
    "Pus6FkNode",
]
