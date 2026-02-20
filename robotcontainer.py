from __future__ import annotations
import math

import commands2
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand, ConditionalCommand, SelectCommand
from commands2.button import CommandGenericHID
from rev import SparkMax
from wpilib import XboxController, Servo, DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Translation3d, Rotation3d

from commands.drive_towards_object import SwerveTowardsObject
from commands.shooting import Shooter, GetReadyAndKeepShooting
from commands.shooting import GetReadyToShoot
from commands.shooting import GetInRange
from commands.shooting import GetReadyToShoot
from commands.aimtodirection import AimToDirection
from commands.trajectory import SwerveTrajectory, JerkyTrajectory
from constants import AutoConstants, DriveConstants, OIConstants
from subsystems import shooter
from subsystems.firing_table import FiringTable
from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics
from subsystems.limelight_camera import LimelightCamera
from subsystems.limelight_localizer import LimelightLocalizer
from subsystems.indexer import Indexer

from commands.reset_xy import ResetXY
from subsystems.photon_tag_camera import PhotonTagCamera
from subsystems.shooter import Shooter
from subsystems.intake import IntakeConstants
from subsystems.intake import Intake
from subsystems.hood import Hood
from subsystems.turret import Turret
from subsystems.intake_arm import IntakeArm

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self, robot) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        # tracks the location of goal posts for shooting, recommends firing angles and speeds
        self.firingTable = FiringTable(
            self.robotDrive,
            shooterLocationOnDrivetrain=Translation2d(x=-0.2, y=0),
            goalIfBlue=Translation2d(x=4.59, y=4.025),
            goalIfRed=Translation2d(x=11.88, y=4.025),
            minimumRangeMeters=2.0,
            maximumRangeMeters=3.0,
        )
        self.firingTableRighWall = FiringTable(
            self.robotDrive,
            shooterLocationOnDrivetrain=Translation2d(x=-0.2, y=0),
            goalIfRed=Translation2d(x=15.0, y=6.0),
            goalIfBlue=Translation2d(x=2, y=2),
            minimumRangeMeters=2.0,
            maximumRangeMeters=3.0,
        )
        self.hoodServo = Servo(
            channel=0
        )
        self.rightIndexer = Indexer(
            leaderCanID=11, leaderInverted= False, followerCanID= 13, followerInverted= False
        )
        self.shooter = Shooter(
            inverted= False,
            hoodServo= self.hoodServo,
        )
        self.turret = Turret(
            leadMotorCANId=70,
            motorClass=SparkMax
        )
        self.intake = Intake(
            inverted= False
        )

        self.limelightLocalizer = LimelightLocalizer(self.robotDrive)

        self.lumaCamera = PhotonTagCamera("luma-front")
        self.centerCamera = LimelightCamera("limelight-center")
        self.limelightthreea = LimelightCamera("limelight-three", isUsb0=True)

        self.limelightLocalizer.addCamera(
            self.lumaCamera,
            cameraPoseOnRobot=Translation3d(x=0.0, y=0.0, z=0.0),  # Translation3d(x=0.0, y=0.155, z=1.0),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(180),
            cameraPitchAngleDegrees=0
        )
        self.limelightLocalizer.addCamera(
            self.centerCamera,
            cameraPoseOnRobot=Translation3d(x=0.4, y=-0.3, z=0.5),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(0.0),
            cameraPitchAngleDegrees=30
        )
        self.pickupCamera = LimelightCamera("limelight-intake")

        self.hood = Hood(leadMotorCANId=43, motorClass= SparkMax)




        # The driver's controller (joystick)
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.configureAutos()

        # Configure default command for driving using joystick sticks
        from commands.holonomicdrive import HolonomicDrive

        # if the driver pushes the left bumper, they drive in "FPV mode" (not field-relative)
        fpvButton = self.driverController.button(XboxController.Button.kLeftBumper)

        self.robotDrive.setDefaultCommand(
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: -0.7 * self.driverController.getRawAxis(XboxController.Axis.kRightX),
                fieldRelative=lambda: not fpvButton.getAsBoolean(),
                deadband=OIConstants.kDriveDeadband,
                rateLimit=True,
                square=True,
            )
        )

        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)


    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # example 1: hold the wheels in "swerve X brake" position, when "X" button is pressed
        brakeCommand = RunCommand(self.robotDrive.setX, self.robotDrive)
        xButton = self.driverController.button(XboxController.Button.kX)
        xButton.whileTrue(brakeCommand)  # while "X" button is True (pressed), keep executing the brakeCommand

        # example 2: when "POV-up" button pressed, reset robot field position to "facing North"
        resetFacingNorthCommand = ResetXY(x=1.0, y=4.0, headingDegrees=0, drivetrain=self.robotDrive)
        povUpButton = self.driverController.povUp()
        povUpButton.whileTrue(resetFacingNorthCommand)

        # example 3: when "POV-down" is pressed, reset robot field position to "facing South"
        resetFacingSouthCommand = ResetXY(x=14.0, y=6.0, headingDegrees=180, drivetrain=self.robotDrive)
        povDownButton = self.driverController.povDown()
        povDownButton.whileTrue(resetFacingSouthCommand)

        from commands.point_towards_location import PointTowardsLocation

        # create a command for keeping the robot nose pointed towards the hub
        keepPointingTowardsHub = PointTowardsLocation(
            drivetrain=self.robotDrive,
            location=Translation2d(x=4.59, y=4.025),
            locationIfRed=Translation2d(x=11.88, y=4.025),
        )
        whenRightTriggerPressed = self.driverController.axisGreaterThan(
            XboxController.Axis.kRightTrigger, threshold=0.5
        )
        whenRightTriggerPressed.whileTrue(keepPointingTowardsHub)
        # ^^ set up a condition for when to do this: do it when the joystick right trigger is pressed by more than 50%

        # create a command for keeping the robot nose pointed 45 degrees (for traversing the hump on a swerve drive)
        keepNoseAt45Degrees = PointTowardsLocation(
            drivetrain=self.robotDrive,
            location=Translation2d(x=999999, y=999999),
            locationIfRed=Translation2d(x=-999999, y=-999999),
        )
        self.driverController.button(XboxController.Button.kRightBumper).whileTrue(keepNoseAt45Degrees)
        # ^^ set up a condition for when to do this: do it when the joystick right bumper is pressed


        getInRange = GetInRange(
            goal=self.firingTable,
            drivetrain=self.robotDrive
        )
        getReady = GetReadyToShoot(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=None,
            drivetrain=self.robotDrive  # if we have a turret, drivetrain=None (otherwise supply drivetrain=self.robotDrive)
        )

        #
        # self.driverController.button(XboxController.Button.kA).whileTrue(
        #     getReady
        # )

        self.driverController.button(XboxController.Button.kA).onTrue(
            InstantCommand(lambda: self.shooter.setVelocityGoal(rpm=2000, rpmTolerance=200))
        )
        self.driverController.button(XboxController.Button.kA).onFalse(
            InstantCommand(lambda: self.shooter.stop())
        )


        from commands.drive_towards_object import SwerveTowardsObject

        # create a command for driving towards one gamepiece, using existing Limelight camera and pipeline 1 inside it
        driveToGamepiece = SwerveTowardsObject(
            drivetrain=self.robotDrive,
            speed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftTrigger),
            # speed controlled by "left trigger" stick of the joystick
            maxLateralSpeed=1.0,
            camera=self.pickupCamera,
            cameraLocationOnRobot=Pose2d(x=+0.4, y=-0.2, rotation=Rotation2d.fromDegrees(0)),
            # camera located at front-right and tilted 30 degrees to the left
            cameraPipeline=0,  # if pipeline 1 in that camera is setup to do gamepiece detection
            dontSwitchToSmallerObject=True,
        )

        # make a command to repeatedly drive to gamepieces (do it again after one gamepiece reached)
        driveToManyGamepieces = driveToGamepiece.repeatedly()

        # setup a condition for when to run that command
        whenLeftTriggerPressed = self.driverController.axisGreaterThan(
            XboxController.Axis.kLeftTrigger, threshold=0.1
        )

        whenLeftTriggerPressed.whileTrue(driveToManyGamepieces)

        self.driverController.button(XboxController.Button.kA).whileTrue(
            # will this make the hood go towards its zero, until it hits it and hits max current?
            RunCommand(lambda: self.hood.setPositionGoal(-1.0), self.hood)
        ).onFalse(
            InstantCommand(lambda: self.hood.stopAndReset(), self.hood)
        )

        self.driverController.button(XboxController.Button.kB).whileTrue(
            # will this make the hood go towards its zero, until it hits it and hits max current?
            RunCommand(lambda: self.hood.setPositionGoal(-4.0), self.hood)
        ).onFalse(
            InstantCommand(lambda: self.hood.stopAndReset(), self.hood)
        )

        self.driverController.button(XboxController.Button.kX).whileTrue(
            InstantCommand(lambda: self.hood.forgetZero(), self.hood)
        )



    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.chosenAuto.getSelected()
        return command()

    def configureAutos(self):
        self.chosenAuto = wpilib.SendableChooser()
        # you can also set the default option, if needed
        self.chosenAuto.setDefaultOption("trajectory example", self.getAutonomousTrajectoryExample)
        self.chosenAuto.addOption("left blue", self.getAutonomousLeftBlue)
        self.chosenAuto.addOption("left red", self.getAutonomousLeftRed)
        self.chosenAuto.addOption("Test2", self.getAutonomousTest2Shooting)
        self.chosenAuto.addOption("Depot",self.getAutonmouseSlowintake)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)



    def getAutonomousTest2Shooting(self):
        setStartPose = ConditionalCommand(
            ResetXY(x=1.542, y=4.025, headingDegrees=+180, drivetrain=self.robotDrive),
            ResetXY(x=AutoConstants.kFieldTags.getFieldLength() - 1.542,
                    y=AutoConstants.kFieldTags.getFieldWidth() - 4.025, headingDegrees=0, drivetrain=self.robotDrive),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kBlue
        )
        shootWhenReady = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.rightIndexer,
            indexerSpeed=0.4
        ).withTimeout(seconds=2.0)

        driveToManyGamepieces = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+1.0,
            waypoints=[
                (1.956, 2.217, -65.136),  # start at x=1.0, y=4.0, heading=0 degrees (North)
                (3.069, 0.677, -0.567),  # next waypoint: x=2.5, y=5.0
                (5.540, 0.677, 2.570),  # next waypoint
                  # next waypoint
            ],
            endpoint=(7.170, 0.833, +80),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=False,  # to keep driving onto next command, set =False
        )
        centerintake = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+0.5,
            waypoints=[

            ],
            endpoint=(7.972, 2.476, +80),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
        )

        finalShootingCMD = GetReadyAndKeepShooting(
            firingTable=self.firingTableRighWall,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.rightIndexer,
            indexerSpeed=0.4
        ).withTimeout(2.0)
        startIntake = InstantCommand(lambda: self.intake.setVelocityGoal(2000, 1000))
        stopIntake = InstantCommand(lambda: self.intake.setVelocityGoal(0, 0000))
        command = setStartPose.andThen(shootWhenReady.andThen(startIntake).andThen(driveToManyGamepieces)
        .andThen(centerintake).andThen(finalShootingCMD))
        return command

    def getAutonmouseSlowintake(self):
        setStartPose = ConditionalCommand(
            ResetXY(x=1.542, y=4.025, headingDegrees=+180, drivetrain=self.robotDrive),
            ResetXY(x=AutoConstants.kFieldTags.getFieldLength() - 1.542, y=AutoConstants.kFieldTags.getFieldWidth() - 4.025, headingDegrees=0, drivetrain=self.robotDrive),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kBlue
        )
        shootWhenReady = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.rightIndexer,
            indexerSpeed=0.4
        ).withTimeout(seconds=2.0)

        drivetoball = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+1.0,
            waypoints=[
                (1.542, 4.025, +180),
                (1.728, 4.431, +180),
                (2.422, 5.206, +180),
                (1.728, 5.754, +180)
            ],
            endpoint=(1.361, 5.931, +180),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=False,  # to keep driving onto next command, set =False
        ).andThen(SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+0.5,
            waypoints=[

            ],
            endpoint=(0.623, 5.931,+180),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=True,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
        ))


        shootafterIntake = GetReadyAndKeepShooting(
            firingTable=self.firingTable,
            shooter=self.shooter,
            turret=self.turret,
            drivetrain=None,  # if we have a turret (otherwise supply drivetrain=self.robotDrive)
            indexer=self.rightIndexer,
            indexerSpeed=0.4
        ).withTimeout(seconds=2.0)

        startIntake = InstantCommand(lambda: self.intake.setVelocityGoal(2000, 1000))
        stopIntake = InstantCommand(lambda: self.intake.setVelocityGoal(0, 0000))
        command = (setStartPose.andThen(shootWhenReady).andThen(startIntake).andThen(drivetoball)
                   .andThen(stopIntake).andThen(shootafterIntake))
        return command






    def getAutonomousLeftBlue(self):
        setStartPose = ResetXY(x=0.783, y=6.686, headingDegrees=+60, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(1.0)).andThen(stop)
        return command

    def getAutonomousLeftRed(self):
        setStartPose = ResetXY(x=15.777, y=4.431, headingDegrees=-120, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(2.0)).andThen(stop)
        return command

    def getAutonomousTrajectoryExample(self) -> commands2.Command:
        command = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+1.0,
            waypoints=[
                (1.0, 4.0, 0.0),  # start at x=1.0, y=4.0, heading=0 degrees (North)
                (2.5, 5.0, 0.0),  # next waypoint: x=2.5, y=5.0
                (3.0, 6.5, 0.0),  # next waypoint
                (6.5, 5.0, -90),  # next waypoint
            ],
            endpoint=(6.0, 4.0, -180),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=False,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
        )

        return command

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode ("test dance") to exercise all subsystems
        """

        # example commands that test drivetrain's motors and gyro (our only subsystem)
        turnRight = AimToDirection(degrees=-45, drivetrain=self.robotDrive, speed=0.25)
        turnLeft = AimToDirection(degrees=45, drivetrain=self.robotDrive, speed=0.25)
        backToZero = AimToDirection(degrees=0, drivetrain=self.robotDrive, speed=0.0)

        command = turnRight.andThen(turnLeft).andThen(backToZero)
        return command
