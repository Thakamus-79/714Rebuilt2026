import commands2
from wpilib import Timer

from subsystems.intake import Intake
from subsystems.intake_arm import IntakeArm
from subsystems.intake_arm import Constants as IntakeArmConstants


class PickUpConstants:
    kPickupRollerSpeed = 2000  # rpm
    kEjectRollerSpeed = -2000  # rpm


class PickUp(commands2.Command):
    """
    Usage example:

    pickUp = PickUp(intake=self.intake, arm=self.intake_arm)
    ...

    self.driverController.buttons(XboxController.Button.kA).whileTrue(pickUp)
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

    def start(self):
        # bring the arm down to its lowest position
        self.arm.setPositionGoal(IntakeArmConstants.maxPosition)
        # start the rollers
        self.intake.setVelocityGoal(PickUpConstants.kPickupRollerSpeed, 0.0)

    def end(self, interrupted) -> None:
        # TODO: when this command ends, stop the intake rollers and bring the arm up
        pass


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

    def start(self):
        # TODO: bring the arm down to its lowest position
        # but then set the intake velocity the PickUpConstants.kEjectRollerSpeed (opposite direction)
        pass

    def end(self, interrupted) -> None:
        # TODO: when this command ends, stop the intake rollers and bring the arm up
        pass


class Shake(commands2.Command):
    """
    This command shakes the intake and is likely going to be an energy hog,
    but it will generate vibration that you need to pop the last gamepieces from the hopper.
    """
    def __init__(self, arm: IntakeArm, intervalSeconds=0.25):
        super().__init__()
        assert intervalSeconds > 0
        self.intervalSeconds = intervalSeconds
        self.arm = arm
        self.start = None
        self.addRequirements(arm)

    def initialize(self):
        # at the start, remember arm's target position before this command started
        self.start = self.arm.positionGoal

    def end(self, interrupted: bool):
        # at the end, return the arm to its original target
        self.arm.setPositionGoal(self.start)

    def execute(self) -> None:
        t = Timer.getFPGATimestamp()
        phase = (t / self.intervalSeconds) % 1.0
        # ^^ this phase oscillates between 0.0 and 0.99999, the arm will go up when phase<0.5 and go down otherwise
        if phase < 0.5:
            self.arm.setPositionGoal(0.8 * IntakeArmConstants.minPosition + 0.2 * IntakeArmConstants.maxPosition)
        else:
            self.arm.setPositionGoal(0.2 * IntakeArmConstants.minPosition + 0.8 * IntakeArmConstants.maxPosition)
