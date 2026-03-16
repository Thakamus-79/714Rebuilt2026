#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface

import typing

from rev import SparkMaxSim, SparkFlexSim, SparkRelativeEncoderSim
from wpimath._controls._controls.plant import DCMotor

from subsystems.turret import Constants as TurretConstants
from subsystems.shooter import ShooterConstants

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    """
    Simulates a motor moving something that strikes two limit switches,
    one on each end of the track. Obviously, this is not particularly
    realistic, but it's good enough to illustrate the point
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller

        # Turret
        turret_gearbox = DCMotor.NEO(1)
        self.turret_motor = SparkFlexSim(robot.robotContainer.turret.motor, turret_gearbox)
        self.turret_encoder = SparkRelativeEncoderSim(robot.robotContainer.turret.motor)
        self.turret = robot.robotContainer.turret

        # Shooter
        shooter_gearbox = DCMotor.NEO(1)
        self.shooter_motor = SparkFlexSim(robot.robotContainer.shooter.leadMotor, shooter_gearbox)
        self.shooter_encoder = SparkRelativeEncoderSim(robot.robotContainer.shooter.leadMotor)
        self.shooter = robot.robotContainer.shooter


    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the turret
        if self.turret.zeroFindingAttemptsLeft != 0:
            self.turret.zeroFindingAttemptsLeft = 0
            self.turret_motor.setPosition(0.0)
            self.turret.pidController = self.turret.motor.getClosedLoopController()
            self.turret.setPositionGoal(0.0)
        turret_dv = tm_diff * (self.turret.positionGoal - self.turret_motor.getPosition()) * TurretConstants.kP * 50.0
        self.turret_motor.setVelocity(turret_dv + self.turret_motor.getVelocity() * (1 - 10.0 * tm_diff))
        self.turret_motor.iterate(self.turret_motor.getVelocity(), 12.0, tm_diff)

        # Simulate the shooter
        shooter_dv = (self.shooter.velocityGoal - self.shooter_motor.getVelocity()) * ShooterConstants.kP * 400.0
        self.shooter_motor.setVelocity(shooter_dv + self.shooter_motor.getVelocity())
        self.shooter_motor.iterate(self.shooter_motor.getVelocity(), 12.0, tm_diff)
