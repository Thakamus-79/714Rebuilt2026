from __future__ import annotations
import commands2
from wpilib import DriverStation

from wpimath.geometry import Rotation2d, Pose2d, Translation2d

from constants import AutoConstants


class ResetXY(commands2.Command):
    def __init__(self, x, y, headingDegrees, drivetrain, flipIfRed=False, flipIfBlue=False):
        """
        Reset the starting (X, Y) and heading (in degrees) of the robot to where they should be.
        :param x: X
        :param y: X
        :param headingDegrees: heading (for example: 0 = "North" of the field, 180 = "South" of the field)
        :param drivetrain: drivetrain on which the (X, Y, heading) should be set
        """
        super().__init__()
        self.flipIfRed = flipIfRed
        self.flipIfBlue = flipIfBlue
        self.drivetrain = drivetrain
        self.position = Pose2d(Translation2d(x, y), Rotation2d.fromDegrees(headingDegrees))
        self.addRequirements(drivetrain)

    def initialize(self):
        flip = False
        if self.flipIfRed and DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            flip = True
        if self.flipIfBlue and DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            flip = True

        if flip:
            f = AutoConstants.kFieldTags
            point = Translation2d(f.getFieldLength() - self.position.x, f.getFieldWidth() - self.position.y)
            self.drivetrain.resetOdometry(Pose2d(point, self.position.rotation() + Rotation2d.fromDegrees(180)))
        else:
            self.drivetrain.resetOdometry(self.position)

    def isFinished(self) -> bool:
        return True  # this is an instant command, it finishes right after it initialized

    def execute(self):
        """
        nothing to do here, this is an instant command
        """

    def end(self, interrupted: bool):
        """
        nothing to do here, this is an instant command
        """


class ResetSwerveFront(commands2.Command):
    def __init__(self, drivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self):
        pose = self.drivetrain.getPose()
        self.drivetrain.resetOdometry(pose)

    def isFinished(self) -> bool:
        return True  # this is an instant command, it finishes right after it initialized

    def execute(self):
        """
        nothing to do here, this is an instant command
        """

    def end(self, interrupted: bool):
        """
        nothing to do here, this is an instant command
        """
