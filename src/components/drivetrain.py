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
    
    def move(self, ySpeed, xSpeed, zRotation):
        self.ySpeed = ySpeed
        self.xSpeed = xSpeed
        self.zRotation = zRotation
    
    def execute(self):
        self.drive.driveCartesian(self.ySpeed, -self.xSpeed, -self.zRotation)
