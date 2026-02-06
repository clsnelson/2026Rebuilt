"""
VisionSubsystem!

Reads all cameras, updates camera inputs and throttles, etc.
"""
import math
from typing import Callable

from commands2 import Subsystem
from phoenix6 import units
from pykit.logger import Logger
from wpilib import Alert
from wpimath.geometry import Pose2d

from constants import Constants
from subsystems.vision.io import CameraObservation, VisionIO, VisionObservation, ObservationType, VisionIOLimelight


# pylint: disable=too-few-public-methods
class VisionSubsystem(Subsystem):
    """el visione"""


    def __init__(
        self,
        vision_consumer: Callable[[Pose2d, units.second, tuple[float, float, float] | None], None],
        *ios: VisionIO) -> None:
        super().__init__()

        self.consumer = vision_consumer
        self.io = ios

        # Initialize inputs
        self.inputs: list[VisionIO.VisionIOInputs] = []
        for _ in self.io:
            self.inputs.append(VisionIO.VisionIOInputs())

        # Initialize disconnected alerts
        self.disconnected_alerts: list[Alert] = []
        for cam_io in self.io:
            self.disconnected_alerts.append(
                Alert(f"Camera {cam_io.VisionIOInputs.name} is disconnected.",
                    Alert.AlertType.kWarning)
            )

    # pylint: disable=too-many-locals
    def periodic(self) -> None:
        """Log and send all observations."""
        for idx, (i, inp) in enumerate(zip(self.io, self.inputs)):
            i.update_inputs(inp)
            Logger.processInputs(f"Vision/{inp.name}", self.inputs[idx])

        all_tag_poses = []
        all_robot_poses = []
        all_robot_poses_accepted = []
        all_robot_poses_rejected = []

        for i, camera in enumerate(self.inputs):
            tag_poses = []
            robot_poses = []
            robot_poses_accepted = []
            robot_poses_rejected = []
            for tag_id in camera.tag_ids:
                pose = Constants.FIELD_LAYOUT.getTagPose(tag_id)
                if pose is not None:
                    tag_poses.append(pose)

            for observation in camera.observations:
                reject_pose = (
                    observation.tag_count == 0
                    or (
                    observation.tag_count == 1
                    and observation.ambiguity > Constants.VisionConstants.max_ambiguity
                    )
                    or abs(observation.pose.Z()) > Constants.VisionConstants.max_z_error
                    or observation.pose.X() < 0.0
                    or observation.pose.X() > Constants.FIELD_LAYOUT.getFieldLength()
                    or observation.pose.Y() < 0.0
                    or observation.pose.Y() > Constants.FIELD_LAYOUT.getFieldWidth()
                )

                robot_poses.append(observation.pose)
                if reject_pose:
                    robot_poses_rejected.append(observation.pose)
                else:
                    robot_poses_accepted.append(observation.pose)

                if reject_pose:
                    continue

                std_dev_factor: float = (
                    pow(observation.avg_tag_dist, 2.0) / observation.tag_count
                )
                linear_std = Constants.VisionConstants.linear_std_dev_baseline * std_dev_factor
                if observation.observation == ObservationType.MEGATAG_2:
                    angular_std = math.inf
                else:
                    angular_std = (Constants.VisionConstants.angular_std_dev_baseline *
                                   std_dev_factor)

                self.consumer(
                    observation.pose.toPose2d(),
                    observation.timestamp,
                    [linear_std, linear_std, angular_std]
                )

            Logger.recordOutput(f"Vision/{camera.name}/TagPoses", tag_poses)
            Logger.recordOutput(f"Vision/{camera.name}/RobotPoses", robot_poses)
            Logger.recordOutput(f"Vision/{camera.name}/RobotPosesAccepted", robot_poses_accepted)
            Logger.recordOutput(f"Vision/{camera.name}/RobotPosesRejected", robot_poses_rejected)
            all_tag_poses.extend(tag_poses)
            all_robot_poses.extend(robot_poses)
            all_robot_poses_accepted.extend(robot_poses_accepted)
            all_robot_poses_rejected.extend(robot_poses_rejected)

        Logger.recordOutput("Vision/AllTagPoses", all_tag_poses)
        Logger.recordOutput("Vision/AllRobotPoses", all_robot_poses)
        Logger.recordOutput("Vision/AllRobotPosesAccepted", all_robot_poses_accepted)
        Logger.recordOutput("Vision/AllRobotPosesRejected", all_robot_poses_rejected)

    def set_throttle(self, throttle: int):
        for cam in self.io:
            if isinstance(cam, VisionIOLimelight):
                cam.set_throttle(throttle)
