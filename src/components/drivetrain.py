import wpilib


class Drivetrain:
    '''
        Robot Drivetrain
    '''

    drive: wpilib.drive.MecanumDrive

    def __init__(self):
        self.ySpeed = 0
        self.xSpeed = 0
        self.zRotation = 0

        self.set_drive = True
    
    def move(self, ySpeed, xSpeed, zRotation):
        self.ySpeed = ySpeed
        self.xSpeed = xSpeed
        self.zRotation = zRotation
    
    def enable(self, value):
        self.set_drive = value
    
    def execute(self):
        if self.set_drive is True:
            self.drive.driveCartesian(self.ySpeed, -self.xSpeed, -self.zRotation)
