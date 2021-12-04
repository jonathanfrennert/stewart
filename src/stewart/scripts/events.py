#!/usr/bin/env python3
#
#   Stewart Platform World Events
#
#   Publish:   /stewart/position_cmd   /stewart/velocity_cmd
#

from time import time

from ik import ikin
from qdot_calc import q_dot

from gen_positions import gen_positions
from velocity import init_velocity
from x_calc import get_x

import os

import rospy
import numpy as np
import pandas as pd

from std_msgs.msg import Float32MultiArray

# Import the Spline stuff:
from splines import  CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5



# Initial position and velocities of Stewart platform joints
STEWART_INIT_CONFIG = np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0]).reshape(6,1)

# Radius of sphere that Stewart platform will aim to stop balls from entering
STEWART_PROTECT_RADIUS = 2

# Cartesian coordinate for center of sphere that Stewart platform will aim to stop
# balls from entering
STEWART_PROTECT_CENTER = np.array([0.0, 0.0, 2.0]).reshape(3,1)

# Protection sphere offset so that the stewart platform definitely hits the ball before
# the ball hits the sphere
OFFSET = 0.2

# Time (in seconds) at which the first ball will hit the center of the sphere that the Stewart platform
# is protecting
BALL1_T = 1

# Time (in seconds) at which the first ball will hit the center of the sphere that the Stewart platform
# is protecting
BALL2_T = 2.5


#
#  Generator Class
#
class Generator:
    # Initialize.
    def __init__(self):
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.
        self.pubStewart = rospy.Publisher("/stewart/position_velocity_cmd", Float32MultiArray, queue_size=10)
        self.pubBall1 = rospy.Publisher("/ball1/position_velocity_cmd", Float32MultiArray, queue_size=10)
        self.pubBall2 = rospy.Publisher("/ball2/position_velocity_cmd", Float32MultiArray, queue_size=10)
        rospy.sleep(0.25)

        # random position of 2 balls that stewart platform will aim to protect against
        ball1_init_pos, ball2_init_pos = gen_positions(6, 3, 6, 2)

        #ball1_init_pos = np.array([0.5, -4, 2]).reshape(3,1)
        #ball2_init_pos = np.array([-0.5, 6, 2]).reshape(3,1)


        # initial velocities of balls such that they will hit the center of the sphere
        # that the Stewart platform is trying to protect in BALL1_T and BALL2_T seconds
        ball1_init_vel = init_velocity(ball1_init_pos, STEWART_PROTECT_CENTER, BALL1_T)
        ball2_init_vel = init_velocity(ball2_init_pos, STEWART_PROTECT_CENTER, BALL2_T)

        # Points of intersection of ball trajectories and sphere which Stewart platform is protecting
        stewart_ball1_meet, time_meet1 = get_x(ball1_init_pos, ball1_init_vel,  STEWART_PROTECT_RADIUS+OFFSET, STEWART_PROTECT_CENTER)
        stewart_ball2_meet, time_meet2 = get_x(ball2_init_pos, ball2_init_vel, STEWART_PROTECT_RADIUS+OFFSET, STEWART_PROTECT_CENTER)

        # Set initial position and velocities of balls
        ball1Data = list(ball1_init_pos) + list(ball1_init_vel)
        ball1Msg = Float32MultiArray(data=ball1Data)
        self.pubBall1.publish(ball1Msg)

        ball2Data =  list(ball2_init_pos) + list(ball2_init_vel)
        ball2Msg = Float32MultiArray(data=ball2Data)
        self.pubBall2.publish(ball2Msg)

        self.segments = (Goto(STEWART_INIT_CONFIG, stewart_ball1_meet, time_meet1, 'Joint'),
                         Goto(stewart_ball1_meet, STEWART_INIT_CONFIG, (time_meet2 - time_meet1) / 2, 'Joint'),
                         Goto(STEWART_INIT_CONFIG, stewart_ball2_meet, (time_meet2 - time_meet1) / 2, 'Joint'),
                         Goto(stewart_ball2_meet, STEWART_INIT_CONFIG, 1, 'Joint'),
                         Hold(STEWART_INIT_CONFIG, 100, 'Joint'))

        # Initialize the current segment index and starting time t0.
        self.index = 0
        self.t0    = 0.0

        self.data = []
        self.sent = False


    # Update is called every 10ms!
    def update(self, t, dt):

        dur = self.segments[self.index].duration()
        if (t - self.t0 >= dur):
            self.t0    = (self.t0 + dur)
            self.index = (self.index + 1)  # not cyclic!
            #self.index = (self.index + 1) % len(self.segments)  # cyclic!

        # Check whether we are done with all segments.
        if (self.index >= len(self.segments)):
            rospy.signal_shutdown("Done with motion")
            return

        (s, sdot) = self.segments[self.index].evaluate(t - self.t0)

        # Collect and send the JointState message.
        posVelData = ikin(s) + list(q_dot(s, sdot))

        if not self.sent:
            if t < 10:
                # for data collection
                self.data.append([t] + posVelData)
            else:
                pd.DataFrame(data=self.data,
                             columns = ['time', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q1dot', 'q2dot', 'q3dot', 'q4dot', 'q5dot', 'q6dot']).to_csv('posVelData.csv')
                self.sent = True

        rosMsg = Float32MultiArray(data=posVelData)
        self.pubStewart.publish(rosMsg)


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('events')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    generator = Generator()

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).
    if False:
        # Original timing (measured/variable time step).
        starttime = rospy.Time.now()
        while not rospy.is_shutdown():

            # Current time (since start)
            servotime = rospy.Time.now()
            t = (servotime - starttime).to_sec()

            # Update the controller.
            generator.update(t, dt)

            # Wait for the next turn.  The timing is determined by the
            # above definition of servo.
            servo.sleep()

    else:
        # Enforcing a constant time step.
        t = 0
        while not rospy.is_shutdown():

            # Update the controller.
            generator.update(t, dt)

            # Wait for the next turn.  The timing is determined by the
            # above definition of servo.
            servo.sleep()

            # Update the time.
            t += dt
