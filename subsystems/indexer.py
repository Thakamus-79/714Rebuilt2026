from commands2 import Subsystem
from rev import SparkBaseConfig, SparkBase, SparkFlex, SparkMax, ResetMode, PersistMode
from wpilib import SmartDashboard


class IndexerConstants:
    kFeederMotorCANID = 25
    kTurntableMotorCANID = 26

    kTargetFeederVelocity = 1000.0,  # RPM (please calibrate!)

    kFF = 18.5 / 10000
    kPFeeder = 0.5 / 10000
    kPTurntable = 0.5 / 10000
    maxRPM = 6000

    kFeederCurrentLimit = 20  # amps, and it must be integer for Rev
    kTurntableCurrentLimit = 20  # amps


class Indexer(Subsystem):
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
    def __init__(self, motorClass=SparkMax) -> None:
        super().__init__()

        self.feederMotor = SparkFlex(IndexerConstants.kFeederMotorCANID, SparkBase.MotorType.kBrushless)
        self.feederMotor.configure(
            _motorConfig(IndexerConstants.kFF, IndexerConstants.kPFeeder, IndexerConstants.kFeederCurrentLimit),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        self.feederController = self.feederMotor.getClosedLoopController()
        self.feederEncoder = self.feederMotor.getEncoder()
        self.feederVelocityGoal = 0.0

        self.turntableMotor = motorClass(IndexerConstants.kTurntableMotorCANID, SparkBase.MotorType.kBrushless)
        self.turntableMotor.configure(
            _motorConfig(IndexerConstants.kFF, IndexerConstants.kPTurntable, IndexerConstants.kTurntableCurrentLimit),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        self.turntableController = self.turntableMotor.getClosedLoopController()
        self.turntableEncoder = self.turntableMotor.getEncoder()
        self.turntableVelocityGoal = 0.0


    def feedGamepieceIntoShooter(self):
        self.setFeederVelocityGoal(IndexerConstants.kTargetFeederVelocity)
        # and maybe set the turntable velocity goal?


    def setFeederVelocityGoal(self, rpm):
        self.feederVelocityGoal = max(-IndexerConstants.maxRPM, min(IndexerConstants.maxRPM, rpm))
        self.feederController.setReference(self.feederVelocityGoal, SparkBase.ControlType.kVelocity)

    def setTurntableVelocityGoal(self, rpm):
        raise NotImplementedError("TO DO")


    def getFeederVelocity(self):
        return self.feederEncoder.getVelocity()

    def getTurntableVelocity(self):
        raise NotImplementedError("TO DO")


    def getFeederVelocityGoal(self):
        return self.feederVelocityGoal

    def getTurntableVelocityGoal(self):
        raise NotImplementedError("TO DO")


    def stop(self):
        self.feederMotor.stopMotor()
        self.feederVelocityGoal = 0
        # TO DO: add the stopping of the turntable


    def periodic(self):
        SmartDashboard.putNumber("IndexerFeeder/rpmSeen", self.getFeederVelocity())
        SmartDashboard.putNumber("IndexerFeeder/rpmGoal", self.getFeederVelocityGoal())
        # TO DO: add the similar things for the turntable


def _motorConfig(kFF, kP, currentLimit) -> SparkBaseConfig:
    config = SparkBaseConfig()
    config.inverted(True)
    config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    config.limitSwitch.forwardLimitSwitchEnabled(False)
    config.limitSwitch.reverseLimitSwitchEnabled(False)
    config.closedLoop.pid(kP, 0.0, 0.0)
    config.closedLoop.velocityFF(kFF)
    config.closedLoop.outputRange(-1, +1)
    config.smartCurrentLimit(currentLimit)
    return config
