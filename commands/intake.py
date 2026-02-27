import commands2

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

    def end(self) -> None:
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

    def end(self) -> None:
        # TODO: when this command ends, stop the intake rollers and bring the arm up
        pass
