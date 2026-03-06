from commands2 import Subsystem
from rev import SparkBaseConfig, SparkBase, SparkFlex, SparkMax, ResetMode, PersistMode
from wpilib import SmartDashboard


class IndexerConstants:
    kFeederMotorCANID = 10
    kSecondFeederMotorCANID = 11
    kWashingMachineCANID = 26

    kTargetFeederVelocity = 1000.0,  # RPM (please calibrate!)

    kFF = 21.8 / 10000
    kPFeeder = 0.8 / 10000
    kPTurntable = 0.5 / 10000
    maxRPM = 6000

    kFeederCurrentLimit = 40  # amps, and it must be integer for Rev
    kTurntableCurrentLimit = 20  # amps


class Indexer(Subsystem):
    """
    The easiest way to test the shooter is to put this into configureButtonBindings():
    ```

    self.driverController.button(XboxController.Button.kA).onTrue(
            InstantCommand(lambda: self.indexer.setFeederVelocityGoal(2000))
    ).onFalse(
            InstantCommand(lambda: self.indexer.stop())
    )

    ```

    """
    def __init__(self, motorClass=SparkMax) -> None:
        super().__init__()

        self.feederMotor1 = SparkFlex(IndexerConstants.kFeederMotorCANID, SparkBase.MotorType.kBrushless)
        self.feederMotor1.configure(
            _motorConfig(IndexerConstants.kFF, IndexerConstants.kPFeeder, IndexerConstants.kFeederCurrentLimit),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        self.feederController1 = self.feederMotor1.getClosedLoopController()
        self.feederEncoder1 = self.feederMotor1.getEncoder()

        self.feederMotor2 = SparkFlex(IndexerConstants.kSecondFeederMotorCANID, SparkBase.MotorType.kBrushless)
        self.feederMotor2.configure(
            _motorConfig(IndexerConstants.kFF, IndexerConstants.kPFeeder, IndexerConstants.kFeederCurrentLimit),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        self.feederController2 = self.feederMotor2.getClosedLoopController()

        self.feederVelocityGoal = 0.0

        self.washingMachineMotor = motorClass(IndexerConstants.kWashingMachineCANID, SparkBase.MotorType.kBrushless)
        self.washingMachineMotor.configure(
            _motorConfig(IndexerConstants.kFF, IndexerConstants.kPTurntable, IndexerConstants.kTurntableCurrentLimit),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        self.washingMachineController = self.washingMachineMotor.getClosedLoopController()
        self.washingMachineEncoder = self.washingMachineMotor.getEncoder()
        self.washingMachineVelocityGoal = 0.0


    def feedGamepieceIntoShooter(self):
        self.setFeederVelocityGoal(IndexerConstants.kTargetFeederVelocity)
        # and maybe set the turntable velocity goal?


    def setFeederVelocityGoal(self, rpm):
        self.feederVelocityGoal = max(-IndexerConstants.maxRPM, min(IndexerConstants.maxRPM, rpm))
        self.feederController1.setReference(self.feederVelocityGoal, SparkBase.ControlType.kVelocity)
        self.feederController2.setReference(-self.feederVelocityGoal, SparkBase.ControlType.kVelocity)

    def setWashingMachineVelocityGoal(self, rpm):
        self.washingMachineVelocityGoal = max(IndexerConstants.maxRPM, min(IndexerConstants.maxRPM, rpm))
        self.washingMachineController.setReference(self.washingMachineVelocityGoal, SparkBase.ControlType.kVelocity)


    def getFeederVelocity(self):
        return self.feederEncoder1.getVelocity()

    def getWashingMachineVelocity(self):
        return self.washingMachineEncoder.getVelocity()


    def getFeederVelocityGoal(self):
        return self.feederVelocityGoal

    def getWashingMachineVelocityGoal(self):
        return self.washingMachineVelocityGoal


    def stop(self):
        self.feederMotor1.stopMotor()
        self.feederMotor2.stopMotor()
        self.feederVelocityGoal = 0
        self.washingMachineMotor.stopMotor()
        self.washingMachineVelocityGoal = 0


    def periodic(self):
        SmartDashboard.putNumber("IndexerFeeder/rpmSeen", self.getFeederVelocity())
        SmartDashboard.putNumber("IndexerFeeder/rpmGoal", self.getFeederVelocityGoal())
        SmartDashboard.putNumber("IndexerFeeder/current", self.feederMotor1.getOutputCurrent())
        SmartDashboard.putNumber("WashingMachine/rpmSeen", self.getWashingMachineVelocity())
        SmartDashboard.putNumber("WashingMachine/rpmGoal", self.getWashingMachineVelocityGoal())
        SmartDashboard.putNumber("WashingMachine/current", self.washingMachineMotor.getOutputCurrent())
        # TO DO: add the similar things for the turntable


def _motorConfig(kFF, kP, currentLimit) -> SparkBaseConfig:
    config = SparkBaseConfig()
    config.inverted(False)
    config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    config.limitSwitch.forwardLimitSwitchEnabled(False)
    config.limitSwitch.reverseLimitSwitchEnabled(False)
    config.closedLoop.pid(kP, 0.0, 0.0)
    config.closedLoop.velocityFF(kFF)
    config.closedLoop.outputRange(-1, +1)
    config.smartCurrentLimit(currentLimit)
    return config
