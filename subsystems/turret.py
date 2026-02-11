# constants right here, to simplify
from commands2 import Subsystem
from rev import SparkBaseConfig, LimitSwitchConfig, SparkBase, SparkMax, ResetMode, PersistMode, ClosedLoopConfig, \
    FeedbackSensor
from wpilib import SmartDashboard, RobotState
from wpimath.filter import SlewRateLimiter


class Constants:
    # other settings
    findingZeroSpeed = 0.14
    stallCurrentLimit = 12  # amps (must be an integer for Rev)
    findingZeroCurrentLimit = stallCurrentLimit * 0.7

    # calibrating? (at first, set it =True and calibrate all the constants above)
    calibrating = False

    # to calibrate, set calibrating = True above, and add this at the end of configureButtonBindings(...) in robotcontainer.py
    # self.driverController.button(XboxController.Button.kA).whileTrue(
    #     # will this make the hood go towards its zero, until it hits it and hits max current?
    #     RunCommand(lambda: self.hood.drive(speed=-0.1), self.hood)
    # ).onFalse(
    #     InstantCommand(lambda: self.hood.stopAndReset(), self.hood)
    # )
    #
    # after this is added, add "Hood" in Elastic, and see what kind of Hood/current is observed
    # when hitting the hard limit at speed -0.1 or you chosen speed
    # (set findingZeroCurrentLimit to half of that value, set calibrating=False and your hood is ready)

    # which range of motion we want from this hood?
    minPosition = -8.0  # motor revolutions
    maxPosition = -0.5  # motor revolutions
    positionTolerance = 0.0625  # motor revolutions

    # calibrated angles:
    minPositionDegrees = -20  # how many degrees is the shooter's heading when turret is @ minPosition?
    maxPositionDegrees = 320  # how many degrees is the shooter's heading when turret is @ maxPosition

    # PID configuration (after you are done with calibrating=True)
    kP = 0.4  # at first make it very small like this, then start tuning by increasing from there
    kD = 0.0  # at first start from zero, and when you know your kP you can start increasing kD from some small value >0
    kMaxOutput = 1.0

    kDegreesPerRotation = (maxPositionDegrees - minPositionDegrees) / (maxPosition - minPosition)
    kRotationsPerDegree = (maxPosition - minPosition) / (maxPositionDegrees - minPositionDegrees)
    initialPositionGoal = maxPosition if abs(maxPosition) < abs(minPosition) else minPosition   # closest to zero
    sign = +1 if maxPositionDegrees > minPositionDegrees else -1


assert Constants.minPositionDegrees != Constants.maxPositionDegrees
assert abs(Constants.minPositionDegrees - Constants.maxPositionDegrees) < 360, "turret range of motion cannot be 360 or more degrees"
assert Constants.minPosition < Constants.maxPosition
assert Constants.maxPosition < 0 or Constants.minPosition > 0, "min/max position should either be both positive or both negative"


class Turret(Subsystem):
    def __init__(
        self,
        leadMotorCANId: int,
        motorClass=SparkMax,
    ) -> None:
        """
        Constructs a hood.
        Please be very, very careful with setting kP and kD in Constants (it's as dangerous as arms and elevators)
        """
        super().__init__()

        self.zeroFound = False
        self.positionGoal = None

        # initialize the motors and switches
        self.motor = motorClass(
            leadMotorCANId, SparkBase.MotorType.kBrushless
        )
        leadMotorConfig = _getLeadMotorConfig(
            inverted=False,
            relPositionFactor=1.0,
        )
        self.motor.configure(
            leadMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters)
        self.motor.clearFaults()

        # initialize pid controller and encoder(s)
        self.pidController = None
        self.relativeEncoder = self.motor.getEncoder()  # this encoder can be used instead of absolute, if you know!

        # the logic of finding the zero needs to be a little smooth
        self.findingZeroRateLimiter = SlewRateLimiter(rateLimit=1.0 * Constants.findingZeroSpeed)
        self.findingZeroRateLimiter.reset(0.0)

        # set the initial hood goal to be the minimum
        self.setPositionGoal(Constants.initialPositionGoal)


    def forgetZero(self):
        self.zeroFound = False
        self.pidController = None
        self.stopAndReset()


    def notReady(self) -> str:
        if not self.zeroFound:
            return "turret zero not found"
        elif abs(self.positionGoal - self.getPosition()) > Constants.positionTolerance:
            return "turret not at target angle"
        else:
            return ""


    def setAngleGoal(self, goalDegrees: float):
        goalRotations = toRotations(goalDegrees)
        # only set the rotation goal if it is between minimum and maximum allowed (otherwise stop and wait)
        if Constants.minPosition < goalRotations < Constants.maxPosition:
            self.setPositionGoal(goalRotations)
        else:
            self.stopAndReset()


    def setPositionGoal(self, goal: float) -> None:
        if goal < Constants.minPosition:
            goal = Constants.minPosition
        if goal > Constants.maxPosition:
            goal = Constants.maxPosition
        self.positionGoal = goal
        if self.pidController is not None:
            self.pidController.setReference(goal, SparkBase.ControlType.kPosition)


    def getPositionGoal(self) -> float:
        return self.positionGoal


    def getPosition(self) -> float:
        return self.relativeEncoder.getPosition()


    def getVelocity(self) -> float:
        return self.relativeEncoder.getVelocity()


    def drive(self, speed, deadband=0.05, maxSpeedRPS=5.0):
        # 1. driving is not allowed in these situations
        if not self.zeroFound and not Constants.calibrating:
            return  # if we aren't calibrating, zero must be found first (before we can drive)

        # 2. speed is assumed to be between -1.0 and +1.0, with a deadband
        if abs(speed) < deadband:
            speed = 0

        # 3. use the speed to drive
        if self.pidController is None:
            self.motor.set(speed) # if we don't we have a PID controller, we use a speed setpoint
        elif speed != 0: # if we have a PID controller, we control the position goal instead
            self.setPositionGoal(self.positionGoal + speed * maxSpeedRPS / 50.0)  # we have 50 decisions/sec


    def stopAndReset(self) -> None:
        self.motor.stopMotor()
        self.motor.clearFaults()
        self.findingZeroRateLimiter.reset(0.0)


    def findZero(self):
        # did we find the zero previously?
        if self.zeroFound:
            return
        # are we calibrating the directions?
        if Constants.calibrating:
            return
        # did we find the zero just now?
        if self.motor.getOutputCurrent() > Constants.findingZeroCurrentLimit:
            self.zeroFound = True
            self.stopAndReset()  # because the zero is found
            self.relativeEncoder.setPosition(0.0)  # found the zero position
            self.pidController = self.motor.getClosedLoopController()
            self.setPositionGoal(Constants.initialPositionGoal)
            return
        # otherwise, continue finding it
        if RobotState.isEnabled():
            speed = self.findingZeroRateLimiter.calculate(Constants.findingZeroSpeed)
            SmartDashboard.putNumber("Turret/findingSpeed", speed)
            self.motor.set(speed)
        else:
            self.findingZeroRateLimiter.reset(0.0)
            SmartDashboard.putNumber("Turret/findingSpeed", 0.0)
            self.motor.set(0)


    def getState(self) -> str:
        if not self.zeroFound:
            return "finding"
        # otherwise, everything is ok
        return "ok"


    def periodic(self):
        # 1. do we need to find zero?
        if not self.zeroFound:
            self.findZero()
        # 2. report to the dashboard
        SmartDashboard.putString("Turret/state", self.getState())
        SmartDashboard.putNumber("Turret/current", self.motor.getOutputCurrent())
        SmartDashboard.putNumber("Turret/goal", self.getPositionGoal())
        SmartDashboard.putNumber("Turret/pos", self.getPosition())
        SmartDashboard.putNumber("Turret/degreesGoal", toDegrees(self.getPositionGoal()))
        SmartDashboard.putNumber("Turret/degreesPos", toDegrees(self.getPosition()))


def _getLeadMotorConfig(
    inverted: bool,
    relPositionFactor: float
) -> SparkBaseConfig:
    config = SparkBaseConfig()
    config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    config.limitSwitch.reverseLimitSwitchEnabled(False)
    config.limitSwitch.forwardLimitSwitchEnabled(False)
    config.encoder.positionConversionFactor(relPositionFactor)
    config.encoder.velocityConversionFactor(relPositionFactor / 60)  # 60 seconds per minute
    config.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
    config.closedLoop.pid(Constants.kP, 0.0, Constants.kD)
    config.closedLoop.velocityFF(0.0)
    config.closedLoop.outputRange(-Constants.kMaxOutput, +Constants.kMaxOutput)
    config.smartCurrentLimit(Constants.stallCurrentLimit)
    config.inverted(inverted)
    return config


def toDegrees(rotations):
    return Constants.minPositionDegrees + (rotations - Constants.minPosition) * Constants.kDegreesPerRotation


def toRotations(degrees):
    degreesAwayFromMin = (degrees - Constants.minPositionDegrees) * Constants.sign

    # if we want to turn +380 degrees, it's same thing as +20 => use the modulo (%) to get the +20 out of +380
    degreesAwayFromMin = degreesAwayFromMin % 360  # this also works if you want -10 degrees: (-10) % 360 = 350

    return Constants.minPosition + degreesAwayFromMin * Constants.sign * Constants.kRotationsPerDegree
