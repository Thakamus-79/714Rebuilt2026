from commands2 import Subsystem

from phoenix6.configs import CANrangeConfiguration
from phoenix6.hardware import CANrange
from phoenix6.signals import UpdateModeValue

from libgrapplefrc import LaserCAN, LaserCanTimingBudget, LaserCanRangingMode, LaserCanRoi

from rev import SparkMax, SparkBase, SparkBaseConfig, ResetMode, PersistMode
from wpilib import SmartDashboard


SHOW_LASERCAN = False
SHOW_CANRANGE = True


class Constants:
    stallCurrentLimit = 5


class Indexer(Subsystem):
    def __init__(self, leaderCanID, leaderInverted=True, followerCanID=None, followerInverted=False) -> None:
        super().__init__()

        # 1. setup the leader motor
        self.motor = SparkMax(leaderCanID, SparkBase.MotorType.kBrushless)

        self.motorConfig = SparkBaseConfig()
        self.motorConfig.inverted(leaderInverted)
        self.motorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        self.motorConfig.smartCurrentLimit(Constants.stallCurrentLimit)
        self.motor.configure(self.motorConfig,
                             ResetMode.kResetSafeParameters,
                             PersistMode.kPersistParameters)

        # when the gamepiece is fully in, it will touch the limit switch -- physical or optical
        self.limitSwitch = self.motor.getForwardLimitSwitch()

        # 2. setup the follower motor, if followerCanID is not None
        self.followerMotor = None
        if followerCanID is not None:
            self.followerMotor = SparkMax(followerCanID, SparkBase.MotorType.kBrushless)
            followerConfig = SparkBaseConfig()
            followerConfig.follow(leaderCanID, leaderInverted != followerInverted)
            followerConfig.smartCurrentLimit(Constants.stallCurrentLimit)
            self.followerMotor.configure(followerConfig,
                                         ResetMode.kResetSafeParameters,
                                         PersistMode.kPersistParameters)

        # 3. safe initial state
        self._setSpeed(0.0)

        # Grapple rangefinder usage ("LaserCAN")
        self.grappleRangeFinder = LaserCAN(0)
        self.grappleRangeFinder.set_timing_budget(LaserCanTimingBudget.TB33ms)
        # ^^ possibilities TB20ms, TB33ms, TB50ms, TB100ms
        self.grappleRangeFinder.set_range(LaserCanRangingMode.Short)
        # ^^ possibilities: Short, Long
        self.grappleRangeFinder.set_roi(LaserCanRoi(x=8, y=8, h=16, w=16))  # this is the default, widest region
        # ^^ possibilities: w/h = width and height (from 1 to 16), x,y = center of the region


        # CTRE rangefinder ("CANrange") usage:
        self.ctrRangeFinder = CANrange(device_id=10)
        # step A: configure the rangefinder
        rangeFinderConfig = CANrangeConfiguration()
        # - set to 100Hz, short range mode
        rangeFinderConfig.to_f_params.update_mode = UpdateModeValue.SHORT_RANGE100_HZ
        # - you can also set the field-of-view for this range finder (narrow angle? or wide angle? in between?)
        # rangeFinderConfig.fov_params.fov_range_x =
        # rangeFinderConfig.fov_params.fov_range_y =
        self.ctrRangeFinder.configurator.apply(rangeFinderConfig)

        # step B: reset the sensor errors before using it, and then it's ready
        self.ctrRangeFinder.clear_sticky_faults()


    def feedGamepieceIntoShooter(self, speed=1.0):
        """
        Rush the gamepiece forward into the shooter, possibly at full speed (100%)
        """
        self._setSpeed(speed)


    def ejectGamepieceBackward(self, speed=0.25, speed2=None):
        """
        Eject the gamepiece back out of the indexer
        """
        self._setSpeed(-speed)


    def stop(self):
        self._setSpeed(0)


    def _setSpeed(self, speed):
        self.motor.set(speed)
        SmartDashboard.putNumber("Indexer/speedGoal", speed)


    def periodic(self) -> None:
        if SHOW_LASERCAN:
            m = self.grappleRangeFinder.get_measurement()
            if m is not None and m.status == 0:  # 0 means "in range", 4 means "out of range", and other values in the docs
                SmartDashboard.putNumber("Indexer/gDistValue", m.distance_mm / 1000.0)
                SmartDashboard.putNumber("Indexer/gAmbient", m.ambient)
                # we also have m.mode and (m.roi.x, m.roi.y, m.roi.w, m.roi.h) if needed
            else:  # 0 means "in range", 4 means "out of range", and other values in the docs
                SmartDashboard.putNumber("Indexer/gDistValue", float('nan'))
                SmartDashboard.putNumber("Indexer/gAmbient", float('nan'))

        if SHOW_CANRANGE:
            distance = self.ctrRangeFinder.get_distance()
            signalStrength = self.ctrRangeFinder.get_signal_strength()
            SmartDashboard.putNumber("Indexer/cDistValue", distance.value)
            SmartDashboard.putString("Indexer/cDistStatus", distance.status.name)
            SmartDashboard.putNumber("Indexer/cDistSignalStrength", signalStrength.value)
            SmartDashboard.putString("Indexer/cDistSignalStatus", signalStrength.status.name)
