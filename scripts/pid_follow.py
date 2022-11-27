
import os
import time
import matplotlib.pyplot as plt
from simple_pid import PID
import math
import json

from rospy import Publisher, Subscriber
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class TurtlebotPID:
    """
    Turtlebot PID
    """

    def __init__(self, dist_setpoint, ang_setpoint, max_speed, max_ang_speed):

        # distnce from subject
        # ranges from 0 to inf (meters) 
        self.r = dist_setpoint
        self.rpid = PID(10, 0.01, 0.1, setpoint=dist_setpoint)
        self.rpid.output_limits = (-max_speed, max_speed)

        # angle distance of the subject from being in the center of the Turtlebot line of sight
        # Ranges from -pi to pi (radians)
        self.the = ang_setpoint
        self.tpid = PID(10, 0.01, 0.1, setpoint=ang_setpoint)
        self.tpid.output_limits = (-max_ang_speed, max_ang_speed)
        
        # TODO: This timer is not in use. the pid is relying on regular subscriptions from
        # ROS sensor topics to operate. If that is not reliable. Can use this class timer
        self.time = time.time()

    def state_update(self, coord: dict, coord_sys = 'polar'):
        """update the state of the Turtlebot relative to the subject

        Args:
            coord (dict): contains either 'r' and 'the' keys for polar coordinates 
                or 'x' and 'y' keys for cartesian coordinates.
                x and y are for axial and lateral movements respectively if using 
                cartesian coordinates.

            coord_sys (str, optional): Relative coordinate system: either polar or cartesian. 
                Defaults to 'polar'.

        Raises:
            ValueError: If invalid coordinate system is specified
        """
        if coord_sys == 'polar':
            self.the = coord['the']
            self.r = coord['r']
        elif coord_sys == 'cartesian':
            self.the = math.atan2(coord['y'], coord['x'])
            self.r = math.sqrt(coord['y']**2 + coord['x']**2)
        else:
            raise ValueError('Turtlebot follow state must either be cartesian or polar')

    def correction_action(self):
        dr_dt = self.rpid(self.r)
        dthe_dt = self.tpid(self.the)
        # publish cmd_vel with dr_dt, dthe_dt
        return (dr_dt, dthe_dt)
        
    def simtest_update(self, lin_speed, ang_speed, dt):
        friction_coeff = 0.8
        drift = 0.05

        self.r += lin_speed * friction_coeff * dt + drift * dt
        self.the += ang_speed * friction_coeff * dt

def pid_callback(data):
    location: dict = json.loads(data.data)
    pid_controller.state_update(location)
    lin_vel, ang_vel = pid_controller.correction_action()
    msg = Twist(linear=lin_vel, angular=ang_vel)
    pub.publish(msg)

def test_main():
    bot = TurtlebotPID(3, -math.pi/3, 0.25, math.pi/6)

    start_time = time.time()
    last_time = start_time

    # Keep track of values for plotting
    rset, tset, r, the = [], [], [], []
    t = []

    # simulate changing the setpoint after 1 second
    # then running pid for 15 seconds
    while time.time() - start_time < 15:
        current_time = time.time()
        dt = current_time - last_time

        correction = bot.correction_action()
        bot.simtest_update(*correction, dt)

        t += [current_time - start_time]
        r += [bot.r]
        the += [bot.the]

        rset += [bot.rpid.setpoint]
        tset += [bot.tpid.setpoint]

        if current_time - start_time > 1:
            bot.rpid.setpoint = 2
            bot.tpid.setpoint = 0

        last_time = current_time

    fig, (ax1, ax2) = plt.subplots(1, 2)
    fig.suptitle('Dummy bot test PID response')
    ax1.plot(t, r, label='measured')
    ax1.plot(t, rset, label='setpoint')

    ax2.plot(t, the, label='measured')
    ax2.plot(t, tset, label='setpoint')

    ax1.set_xlabel('time')
    ax1.set_ylabel('distance')
    ax1.legend()

    ax2.set_xlabel('time')
    ax2.set_ylabel('distance')
    ax2.legend()
    
    if os.getenv('NO_DISPLAY'):
        fig.savefig(f"Tbot_test_PID.png")
    else:
        plt.show()
    exit()

if __name__ == '__main__':
    if False: test_main()
    rospy.init_node('pid_follower')
    pid_controller = TurtlebotPID(1.5, 0, 0.25, math.pi/6)
    pub = Publisher('/cmd_vel', Twist, queue_size=2)
    sub = Subscriber('/object_location', String, pid_callback)