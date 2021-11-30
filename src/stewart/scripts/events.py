# -*- coding: utf-8 -*-
#!/usr/bin/env python3
#
#   Stewart Platform World Events
#
#   Publish:   /stewart/position_cmd   /stewart/velocity_cmd
#
import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray

# Import the Spline stuff:
from splines import  CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5


#
#  Generator Class
#
class Generator:
    # Initialize.
    def __init__(self):
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.
        self.pub = rospy.Publisher("/stewart/position_cmd", Float32MultiArray, queue_size=10)
        rospy.sleep(0.25)


        # Create the splines (cubic for now, use Goto5() for HW#5P1).
        pi = np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0])
        pf = np.array([0.275, 0.275, 2.908, 0.0, 0.0, 0.0])

        self.segments = (Goto(pi, pf, 0.75, 'Joint'),
                         Goto(pf, pi, 0.75, 'Joint'))

        # Initialize the current segment index and starting time t0.
        self.index = 0
        self.t0    = 0.0


    # Update is called every 10ms!
    def update(self, t, dt):

        dur = self.segments[self.index].duration()
        if (t - self.t0 >= dur):
            self.t0    = (self.t0 + dur)
            #self.index = (self.index + 1)                       # not cyclic!
            self.index = (self.index + 1) % len(self.segments)  # cyclic!

        # Check whether we are done with all segments.
        if (self.index >= len(self.segments)):
            rospy.signal_shutdown("Done with motion")
            return

        (s, sdot) = self.segments[self.index].evaluate(t - self.t0)

        # Collect and send the JointState message.
        cmdmsg = ik(s)
        self.pub.publish(cmdmsg)


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
    # Choose the timing method - this is for HW5P2.
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
