import commands2
from wpilib import Timer, SmartDashboard

from subsystems.intake import Intake
from subsystems.intake_arm import IntakeArm
from subsystems.intake_arm import Constants as IntakeArmConstants


class PickUpConstants:
    kPickupRollerSpeed = -1400  # rpm
    kEjectRollerSpeed = +1400  # rpm


class StowIntake(commands2.Command):
    """
    Usage example:

    pickUp = PickUp(intake=self.intake, arm=self.intake_arm)
    ...

    self.driverController.buttons(XboxController.Button.kA).whileTrue(pickUp)
    """
    def __init__(self, arm: IntakeArm):
        super().__init__()
        self.arm = arm
        self.addRequirements(arm)

    def isFinished(self) -> bool:
        return False  # never finishes on its own (maybe this will change when we install a rangefinder)

    def execute(self) -> None:
        return  # there is nothing to do while the command is running

    def initialize(self):
        # bring the arm down to its lowest position
        self.arm.setPositionGoal(IntakeArmConstants.stowedPosition)

    def end(self, interrupted) -> None:
        # bring the arm down to its lowest position
        self.arm.setPositionGoal(IntakeArmConstants.neutralPosition)


class PickUp(commands2.Command):
    """
    Usage example:

    pickUp = PickUp(intake=self.intake, arm=self.intake_arm)
    ...

    self.driverController.buttons(XboxController.Button.kA).whileTrue(pickUp)
    """
    def __init__(self, intakeRollers: Intake, arm: IntakeArm):
        super().__init__()
        self.intakeRollers = intakeRollers
        self.arm = arm
        self.addRequirements(intakeRollers)
        self.addRequirements(arm)
        SmartDashboard.putString("PickUp", "created")

    def isFinished(self) -> bool:
        return False  # never finishes on its own (maybe this will change when we install a rangefinder)

    def execute(self) -> None:
        return  # there is nothing to do while the command is running

    def initialize(self):
        SmartDashboard.putString("PickUp", "started")
        # bring the arm down to its lowest position
        self.arm.setPositionGoal(IntakeArmConstants.deployedPosition)
        # start the rollers
        self.intakeRollers.setVelocityGoal(PickUpConstants.kPickupRollerSpeed, 0.0)

    def end(self, interrupted) -> None:
        SmartDashboard.putString("PickUp", "finished")
        self.arm.setPositionGoal(IntakeArmConstants.neutralPosition)
        self.intakeRollers.setVelocityGoal(0, 0.0)


class Eject(commands2.Command):
    """
    Usage example:

    eject = Eject(intake=self.intake, arm=self.intake_arm)
    ...

    self.driverController.buttons(XboxController.Button.kB).whileTrue(eject)
    """

    def __init__(self, intake: Intake, arm: IntakeArm):
        super().__init__()
        self.intake = intake
        self.arm = arm
        self.addRequirements(intake)
        self.addRequirements(arm)

    def isFinished(self) -> bool:
        return False  # never finishes on its own (maybe this will change when we install a rangefinder)

    def execute(self) -> None:
        return  # there is nothing to do while the command is running

    def initialize(self):
        # bring the arm down to its lowest position
        self.arm.setPositionGoal(IntakeArmConstants.deployedPosition)
        # start the rollers
        self.intake.setVelocityGoal(-PickUpConstants.kPickupRollerSpeed, 0.0)

    def end(self, interrupted) -> None:
        # bring the arm down to its lowest position
        self.arm.setPositionGoal(IntakeArmConstants.neutralPosition)
        # start the rollers
        self.intake.setVelocityGoal(0, 0.0)


class SuppressIntake(commands2.Command):

    """
    Usage example:

    suppress = SuppressIntake(arm=self.intake_arm)
    ...

    self.driverController.buttons(XboxController.Button.kA).whileTrue(suppress)
    """
    def __init__(self, arm: IntakeArm):
        super().__init__()
        self.arm = arm
        self.addRequirements(arm)

    def isFinished(self) -> bool:
        return True  # never finishes on its own (maybe this will change when we install a rangefinder)

    def execute(self) -> None:
        return  # there is nothing to do while the command is running

    def initialize(self):
        # bring the arm down to its lowest position
        self.arm.setPositionGoal(IntakeArmConstants.stowedPosition)
        print("SuppressIntake")

    def end(self, interrupted) -> None:
        pass


class ShakeIntake(commands2.Command):
    """
    This command shakes the intake and is likely going to be an energy hog,
    but it will generate vibration that you need to pop the last gamepieces from the hopper.
    """
    def __init__(self, intake: Intake, arm: IntakeArm, intervalSeconds=1.5):
        super().__init__()
        assert intervalSeconds > 0
        self.intervalSeconds = intervalSeconds
        self.arm = arm
        self.intake = intake
        self.start = None
        self.startTime = 0.0
        self.addRequirements(arm)
        self.addRequirements(intake)
        SmartDashboard.putString("ShakeIntake", "created")

    def initialize(self):
        # at the start, remember arm's target position before this command started
        SmartDashboard.putString("ShakeIntake", "started")
        self.start = self.arm.positionGoal
        self.startTime = Timer.getFPGATimestamp()
        self.intake.setVelocityGoal(PickUpConstants.kPickupRollerSpeed, 0.1 * PickUpConstants.kPickupRollerSpeed)

    def end(self, interrupted: bool):
        SmartDashboard.putString("ShakeIntake", "finished")
        # at the end, return the arm to its original target
        self.arm.setPositionGoal(IntakeArmConstants.neutralPosition)
        self.intake.stop()

    def execute(self) -> None:
        t = Timer.getFPGATimestamp() - self.startTime
        phase = (t / self.intervalSeconds) % 1.0
        # ^^ this phase oscillates between 0.0 and 0.99999, the arm will go up when phase<0.5 and go down otherwise
        if phase < 0.5:
            self.arm.setPositionGoal(IntakeArmConstants.deployedPosition)
        else:
            self.arm.setPositionGoal(0.75 * IntakeArmConstants.neutralPosition + 0.25 * IntakeArmConstants.stowedPosition)
