import enum

from wpilib.interfaces.generichid import GenericHID

__all__ = ["FightStick"]


class FightStick(GenericHID):
    """
        Handle input from Xbox 360 or Xbox One controllers connected to the Driver Station.

        This class handles Xbox input that comes from the Driver Station. Each time a value is
        requested the most recent value is returned. There is a single class instance for each controller
        and the mapping of ports to hardware buttons depends on the code in the Driver Station.
     """

    class Button(enum.IntEnum):
        kA = 1
        kB = 2
        kX = 3
        kY = 4
        kL1 = 5
        kL2 = 2
        kR1 = 6
        kR2 = 3

    def __init__(self, port: int) -> None:
        """Construct an instance of an XBoxController. The XBoxController index is the USB port on the Driver Station.

        :param port: The port on the Driver Station that the joystick is plugged into
        """
        super().__init__(port)

    def getX(self) -> float:
        """Get the X axis value of the controller.

        :param hand: Side of controller whose value should be returned
        :return: The X axis value of the controller
        """
        return self.getRawAxis(0)

    def getY(self) -> float:
        """Get the Y axis value of the controller.

        :param hand: Side of controller whose value should be returned
        :return: The Y axis value of the controller
        """
        return self.getRawAxis(1)

    def getAButton(self) -> bool:
        """Read the value of the A button on the controller

        :return: The state of the A button
        """
        return self.getRawButton(self.Button.kA)

    def getAButtonPressed(self) -> bool:
        """Whether the A button was pressed since the last check.

        :returns: Whether the button was pressed since the last check.
        """
        return self.getRawButtonPressed(self.Button.kA)

    def getAButtonReleased(self) -> bool:
        """Whether the A button was released since the last check.

        :returns: Whether the button was released since the last check.
        """
        return self.getRawButtonReleased(self.Button.kA)

    def getBButton(self) -> bool:
        """Read the value of the B button on the controller

        :return: The state of the B button
        """
        return self.getRawButton(self.Button.kB)

    def getBButtonPressed(self) -> bool:
        """Whether the B button was pressed since the last check.

        :returns: Whether the button was pressed since the last check.
        """
        return self.getRawButtonPressed(self.Button.kB)

    def getBButtonReleased(self) -> bool:
        """Whether the B button was released since the last check.

        :returns: Whether the button was released since the last check.
        """
        return self.getRawButtonReleased(self.Button.kB)

    def getXButton(self) -> bool:
        """Read the value of the X button on the controller

        :return: The state of the X button
        """
        return self.getRawButton(self.Button.kX)

    def getXButtonPressed(self) -> bool:
        """Whether the X button was pressed since the last check.

        :returns: Whether the button was pressed since the last check.
        """
        return self.getRawButtonPressed(self.Button.kX)

    def getXButtonReleased(self) -> bool:
        """Whether the X button was released since the last check.

        :returns: Whether the button was released since the last check.
        """
        return self.getRawButtonReleased(self.Button.kX)

    def getYButton(self) -> bool:
        """Read the value of the Y button on the controller

        :return: The state of the Y button
        """
        return self.getRawButton(self.Button.kY)

    def getYButtonPressed(self) -> bool:
        """Whether the Y button was pressed since the last check.

        :returns: Whether the button was pressed since the last check.
        """
        return self.getRawButtonPressed(self.Button.kY)

    def getYButtonReleased(self) -> bool:
        """Whether the Y button was released since the last check.

        :returns: Whether the button was released since the last check.
        """
        return self.getRawButtonReleased(self.Button.kY)

    def getL1Button(self) -> bool:
        return self.getRawButton(self.Button.kL1)

    def getL1ButtonPressed(self) -> bool:
        return self.getRawButtonPressed(self.Button.kL1)

    def getL1ButtonReleased(self) -> bool:
        return self.getRawButtonReleased(self.Button.kL1)

    def getR1Button(self) -> bool:
        return self.getRawButton(self.Button.kR1)

    def getR1ButtonPressed(self) -> bool:
        return self.getRawButtonPressed(self.Button.kR1)

    def getR1ButtonReleased(self) -> bool:
        return self.getRawButtonReleased(self.Button.kR1)

    def getL2Button(self) -> bool:
        return self.getRawAxis(self.Button.kL2)

    def getR2Button(self) -> bool:
        return self.getRawAxis(self.Button.kR2)
