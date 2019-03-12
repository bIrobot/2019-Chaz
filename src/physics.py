from pyfrc.physics import drivetrains


class PhysicsEngine(object):
    '''
       Simulates a 4-wheel mecanum robot using Tank Drive joystick control
    '''

    def __init__(self, physics_controller):
        '''
            :param physics_controller: `pyfrc.physics.core.Physics` object
                                       to communicate simulation effects to
        '''

        self.physics_controller = physics_controller
        self.drivetrain = drivetrains.MecanumDrivetrain()

    def update_sim(self, hal_data, now, tm_diff):
        '''
            Called when the simulation parameters for the program need to be
            updated.

            :param now: The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
        '''

        # Simulate the drivetrain
        lr_motor = -hal_data['CAN'][1]['value']
        rr_motor = hal_data['CAN'][2]['value']
        lf_motor = -hal_data['CAN'][3]['value']
        rf_motor = hal_data['CAN'][4]['value']

        vx, vy, vw = self.drivetrain.get_vector(lr_motor, rr_motor, lf_motor, rf_motor)
        self.physics_controller.vector_drive(-vx, vy, vw, tm_diff)
        