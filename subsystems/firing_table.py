#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from typing import List, Iterable

from commands2 import Subsystem
from wpilib import DriverStation, SmartDashboard, SendableChooser
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from subsystems.drivesubsystem import DriveSubsystem
from constants import LookupTable


# TODO : calibrate this lookup table on a real robot, and add more points
RECOMMENDED_SHOOTER_RPM_BY_DISTANCE = LookupTable({
    1.44 : 2375,
    2.4: 2810,
    3.04: 2875,
    3.9 : 3000,
    6.82 : 3750,  # imagination


   # if distance is 12m, spin at 6000 rpm
})

# TODO : calibrate this lookup table on a real robot, and add more points
RECOMMENDED_SHOOTER_HOOD_POSITION_BY_DISTANCE = LookupTable({
    1.44: -0.01,
    2.4: -0.2,
    3.04: -0.4,
    3.9: -0.5,
    6.82: -0.8,  # imagination
    # if distance is 12m, hood position -0.8 (firing very horizontally)
})



class FiringTable(Subsystem):
    rpm: SendableChooser | None = None
    factor: SendableChooser | None = None
    hoodPos: SendableChooser | None = None
    ballVelocity: SendableChooser | None = None

    """
    Tracks how far the goal is, recommends shooter speed (RPM), firing angle and direction
    """
    def __init__(
        self,
        drivetrain: DriveSubsystem,
        shooterLocationOnDrivetrain: Translation2d,
        goalIfBlue: Translation2d,
        goalIfRed: Translation2d,
        fuelStashesIfBlue: Iterable[Translation2d] = (),
        fuelStashesIfRed: Iterable[Translation2d] = (),
        minimumRangeMeters: float = 0.0,
        maximumRangeMeters: float = 0.0,
    ) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.shooterLocationOnDrivetrain = shooterLocationOnDrivetrain
        self.goalIfBlue = goalIfBlue
        self.goalIfRed = goalIfRed
        self.fuelStashesIfBlue = fuelStashesIfBlue
        self.fuelStashesIfRed = fuelStashesIfRed
        self.minimumRangeMeters = minimumRangeMeters
        self.maximumRangeMeters = maximumRangeMeters

        self.goal: Translation2d | None = None
        self.vectorToGoal: Translation2d | None = None
        self.shooterLocation: Translation2d | None = None

        if FiringTable.factor is None:
            FiringTable.factor = SendableChooser()
            for percent in range(80, 100, 1):
                FiringTable.factor.addOption(f'0{percent}%', percent / 100)
            FiringTable.factor.setDefaultOption("100%", 1.00)
            for percent in range(101, 125, 1):
                FiringTable.factor.addOption(f'{percent}%', percent / 100)
            SmartDashboard.putData("FiringTable/factor", FiringTable.factor)

        if FiringTable.rpm is None:
            FiringTable.rpm = SendableChooser()
            FiringTable.rpm.setDefaultOption("lookup", None)
            for rpm in range(2*1000, 2*6000, 125):
                FiringTable.rpm.addOption(str(rpm/2), rpm/2)
            SmartDashboard.putData("FiringTable/rpmChosen", FiringTable.rpm)

        if FiringTable.hoodPos is None:
            FiringTable.hoodPos = SendableChooser()
            FiringTable.hoodPos.setDefaultOption("lookup", None)
            for f in [0.01, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]:
                FiringTable.hoodPos.addOption(str(-f), -f)
            SmartDashboard.putData("FiringTable/hoodPosChosen", FiringTable.hoodPos)

        if FiringTable.ballVelocity is None:
            FiringTable.ballVelocity = SendableChooser()
            FiringTable.ballVelocity.setDefaultOption("6.0", 6.0)
            for f in [1.4, 2.0, 2.8, 4.0, 5.0, 7.0, 8.0, 9.8, 12.0, 18.0, 99.0]:
                FiringTable.ballVelocity.addOption(str(f), f)
            SmartDashboard.putData("FiringTable/ballVelocity", FiringTable.ballVelocity)

        self.resetSmartDashboard()


    def recommendedShooterRpm(self):
        if self.vectorToGoal is None:
            return 0.0
        distanceMeters = self.distance()
        SmartDashboard.putNumber("FiringTable/distance", distanceMeters)

        # lookup the recommended RPM in the table
        rpm = self.rpm.getSelected()
        if rpm is None:
            rpm = RECOMMENDED_SHOOTER_RPM_BY_DISTANCE.interpolate(distanceMeters)

        SmartDashboard.putNumber("FiringTable/rpm", rpm)
        return rpm


    def recommendedFiringHoodPosition(self) -> float | None:
        if self.vectorToGoal is None:
            return None
        distanceMeters = self.distance()

        # lookup the recommended angle in the lookup table
        hoodPosition = self.hoodPos.getSelected()
        if hoodPosition is None:
            hoodPosition = RECOMMENDED_SHOOTER_HOOD_POSITION_BY_DISTANCE.interpolate(distanceMeters)

        SmartDashboard.putNumber("FiringTable/hoodPos", hoodPosition)
        return hoodPosition


    def recommendedTurretDirection(self) -> Rotation2d | None:
        """
        If the goal is 30 degrees left, while the drivetrain is 10 degrees left... the turret should turn +20 degrees
        """
        if self.vectorToGoal is None:
            return None

        drivetrainPose = self.drivetrain.getPose()
        result = self.vectorToGoal.angle() - drivetrainPose.rotation()

        SmartDashboard.putNumber("FiringTable/turretDirDegrees", result.degrees())
        return result


    def vector(self) -> Translation2d | None:
        return self.vectorToGoal


    def distance(self) -> float:
        return self.vectorToGoal.norm() if self.vectorToGoal is not None else 0


    def direction(self) -> Rotation2d:
        return self.vectorToGoal.angle() if self.vectorToGoal is not None else Rotation2d(0)


    def periodic(self):
        alliance = DriverStation.getAlliance()
        pose = self.drivetrain.getPose()
        if alliance == DriverStation.Alliance.kRed:
            self.goal = self.goalIfRed
            if pose.x < 11.91:
                self.goal = self.findNearestStash(pose, self.fuelStashesIfRed) or self.goalIfRed
        else:
            self.goal = self.goalIfBlue
            if pose.x > 4.62:
                self.goal = self.findNearestStash(pose, self.fuelStashesIfBlue) or self.goalIfBlue

        self.shooterLocation = pose.translation() + self.shooterLocationOnDrivetrain.rotateBy(pose.rotation())
        timeOfFlight = (self.goal - self.shooterLocation).norm() / self.ballVelocity.getSelected()

        adjustment = Translation2d(-self.drivetrain.vx * timeOfFlight, -self.drivetrain.vy * timeOfFlight)
        #SmartDashboard.putString("FiringTable/adjustment", str((round(adjustment.x, 2), round(adjustment.y, 2))))
        effectiveGoal = self.goal + adjustment
        if self.drivetrain.field is not None:
            self.drivetrain.field.getObject("effectiveGoal").setPose(Pose2d(effectiveGoal, Rotation2d()))

        self.vectorToGoal = effectiveGoal - self.shooterLocation

        distanceMeters = self.distance()
        SmartDashboard.putNumber("FiringTable/distance", distanceMeters)


    def resetSmartDashboard(self):
        SmartDashboard.putNumber("FiringTable/distance", float('nan'))
        SmartDashboard.putNumber("FiringTable/rpm", float('nan'))
        SmartDashboard.putNumber("FiringTable/hoodPos", float('nan'))
        SmartDashboard.putNumber("FiringTable/turretDirDegrees", float('nan'))


    @staticmethod
    def findNearestStash(pose: Pose2d, stashes: Iterable[Translation2d]) -> Translation2d | None:
        """
        If you have multiple stash points where robot can shoot, find the nearest one to the current position
        """
        start = pose.translation()
        result, distance = None, float('inf')
        for point in stashes:
            d = start.squaredDistance(point)
            if d < distance:
                result = point
                distance = d
        return result
