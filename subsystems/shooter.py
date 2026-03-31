from commands2 import Subsystem
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import Follower, MotionMagicVelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue, InvertedValue, MotorAlignmentValue
from rev import SparkBaseConfig, SparkBase, SparkFlex, ResetMode, PersistMode
from wpilib import SmartDashboard, Servo
from typing import List

from subsystems.hood import Hood


class ShooterConstants:
    kShooterMotorA_CANID = 41
    kShooterMotorB_CANID = 40

    maxRPM = 5700
    kFF = 1.68 / 10000  # 18.74 for 714, 1.68 for 8630  # 18.5 for old Vortexes, go figure
    kP = 8.0 / 10000  # 6.0 for 8630, 8.0 for 714
    kD = 0.0 / 10000

    stallCurrentLimit = 160  # amps, and it must be integer for Rev


class Shooter(Subsystem):
    """
    The easiest way to test the shooter is to put this into configureButtonBindings():
    ```

    self.driverController.button(XboxController.Button.kA).onTrue(
            InstantCommand(lambda: self.shooter.setVelocityGoal(2000, 1000))
    ).onFalse(
            InstantCommand(lambda: self.shooter.stop())
    )

    ```

    """
    def __init__(self, inverted=True, motorClass=SparkFlex, hoodServos: List[Servo | Hood] = ()) -> None:
        super().__init__()

        self.hoodServos = hoodServos
        # just in case the bounds were not set, set them
        for servo in hoodServos:
            if hasattr(servo, "setBounds"):
                servo.setBounds(2000, 1500, 1500, 1500, 1000)
        self.hoodServoGoal = 0.0
        self.setHoodServoGoal(self.hoodServoGoal)

        self.revLeadMotor = None
        self.revFollowMotor = None
        self.revPidController = None
        self.revEncoder = None

        if motorClass == SparkFlex:
            self.revLeadMotor = SparkFlex(ShooterConstants.kShooterMotorA_CANID, SparkBase.MotorType.kBrushless)
            self.revLeadMotor.configure(
                _getLeadMotorConfig(),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )
            self.revFollowMotor = SparkFlex(ShooterConstants.kShooterMotorB_CANID, SparkBase.MotorType.kBrushless)
            self.revFollowMotor.configure(
                _getFollowMotorConfig(),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )
            self.revPidController = self.revLeadMotor.getClosedLoopController()
            self.revEncoder = self.revLeadMotor.getEncoder()

        self.talonLeadMotor = None
        self.talonFollowMotor = None
        self.talonControlRequest = MotionMagicVelocityVoltage(0, acceleration=250, slot=0)

        if motorClass == TalonFX:
            self.talonLeadMotor = TalonFX(ShooterConstants.kShooterMotorA_CANID)
            self.talonFollowMotor = TalonFX(ShooterConstants.kShooterMotorA_CANID)
            leadConfig = TalonFXConfiguration()
            leadConfig.motor_output.neutral_mode = NeutralModeValue.COAST
            leadConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            leadConfig.slot0.k_p = ShooterConstants.kP * 0.55
            leadConfig.slot0.k_i = 0
            leadConfig.slot0.k_d = 0
            leadConfig.slot0.k_v = ShooterConstants.kFF * 0.55
            self.talonLeadMotor.configurator.apply(leadConfig)
            self.talonFollowMotor.set_control(Follower(self.talonLeadMotor.device_id, MotorAlignmentValue.OPPOSED))

        self.velocityGoal = 0
        self.velocityTolerance = 0
        self.inverted = -1 if inverted else 1
        # best is to not invert the lead motor, because such config is lost in brownout

        self.reportedVelocityGoal = 0
        self.reportedVelocitySeen = 0

    def notReady(self) -> str:
        velocity = self.getVelocity()
        if velocity < self.velocityGoal - self.velocityTolerance:
            return f"shooter under velocity goal: {velocity} < {self.velocityGoal}"
        elif velocity > self.velocityGoal + self.velocityTolerance:
            return f"shooter above velocity goal: {velocity} > {self.velocityGoal}"
        else:
            return ""  # shooter is ready

    def setHoodServoGoal(self, goal):
        self.hoodServoGoal = goal
        SmartDashboard.putNumber("Shooter/hoodServoGoal", goal)
        for s in self.hoodServos:
            s.set(self.hoodServoGoal)

    def setVelocityGoal(self, rpm, rpmTolerance):
        self.velocityTolerance = rpmTolerance
        self.velocityGoal = max(-ShooterConstants.maxRPM, min(ShooterConstants.maxRPM, rpm))
        if self.revPidController is not None:
            self.revPidController.setReference(self.velocityGoal * self.inverted, SparkBase.ControlType.kVelocity)
        if self.talonLeadMotor is not None:
            self.talonLeadMotor.set_control(self.talonControlRequest.with_velocity(self.velocityGoal * self.inverted))

    def getVelocity(self):
        if self.revEncoder is not None:
            return self.revEncoder.getVelocity() * self.inverted
        if self.talonLeadMotor is not None:
            return self.talonLeadMotor.get_velocity().value * self.inverted
        return float('nan')

    def getVelocityGoal(self):
        return self.velocityGoal * self.inverted

    def periodic(self):
        seen = self.getVelocity()
        goal = self.getVelocityGoal()
        if goal != self.reportedVelocityGoal or abs(seen - self.reportedVelocitySeen) >= 0.001 * seen:
            SmartDashboard.putNumber("Shooter/rpmSeen", seen)
            self.reportedVelocitySeen = seen
            SmartDashboard.putNumber("Shooter/rpmGoal", goal)
            self.reportedVelocityGoal = goal
        if self.revLeadMotor is not None:
            SmartDashboard.putNumber("Shooter/CurrentL", self.revLeadMotor.getOutputCurrent())
            SmartDashboard.putNumber("Shooter/CurrentF", self.revFollowMotor.getOutputCurrent())
        if self.talonLeadMotor is not None:
            SmartDashboard.putNumber(
                "Shooter/CurrentL",
                self.talonLeadMotor.get_stator_current().value + self.talonLeadMotor.get_supply_current().value
            )
            SmartDashboard.putNumber(
                "Shooter/CurrentF",
                self.talonFollowMotor.get_stator_current().value + self.talonFollowMotor.get_supply_current().value
            )


    def stop(self):
        if self.revLeadMotor is not None:
            self.revLeadMotor.stopMotor()
        if self.talonLeadMotor is not None:
            self.talonLeadMotor.stopMotor()
        self.velocityTolerance = 0
        self.velocityGoal = 0

    def driveHoodServo(self, velocity):
        """
        Usage example in configureButtonBindings(...):
        ```

        # for calibrataion, drive that servo using the right stick of the joystick up/down
        self.shooter.setDefaultCommand(RunCommand(
           lambda: self.shooter.driveHoodServo(
               self.driverController.getRawAxis(XboxController.Axis.kRightY)
           )
        ))

        ```
        :param velocity:
        """
        self.setHoodServoGoal(self.hoodServoGoal + velocity * 0.01)


def _getLeadMotorConfig() -> SparkBaseConfig:
    config = SparkBaseConfig()
    config.inverted(True)
    config.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
    config.limitSwitch.forwardLimitSwitchEnabled(False)
    config.limitSwitch.reverseLimitSwitchEnabled(False)
    config.closedLoop.pid(ShooterConstants.kP, 0.0, ShooterConstants.kD)
    config.closedLoop.velocityFF(ShooterConstants.kFF)
    config.closedLoop.outputRange(-1, +1)
    #config.smartCurrentLimit(ShooterConstants.stallCurrentLimit)
    return config

def _getFollowMotorConfig():
    followConfig = SparkBaseConfig()
    followConfig.follow(ShooterConstants.kShooterMotorA_CANID, True)  # True = inverted when following
    followConfig.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
    #followConfig.smartCurrentLimit(ShooterConstants.stallCurrentLimit)
    return followConfig
