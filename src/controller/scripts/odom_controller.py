#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from tf.transformations import euler_from_quaternion
from controller.msg import twoFloats
from rosgraph_msgs.msg import Clock

home_x, home_y = 0,0
velocity = 0.1
beta = 2
Rth = 0.5
dc = 2.5
delc = 3.14/2.5
dphi = 0.5
k=2
lambda_i = 2


def freeController(V, t, tf, vf):

    omega =  (beta * V[0] / Rth) * np.sign(V[2])
    v = velocity*(1-(1 - vf/velocity)*np.e**(tf-t))
    return omega, v

def engagedController(V, delta, t, te, ve):

    omega = (beta * V[0] / Rth) * np.sign(V[2]) - k * (V[0]/Rth + 2*velocity/dphi) * np.sign(delta)
    v = max(ve - (t-te)*lambda_i*velocity**2/(dc-dphi),0)
    print("engaged:"+str((beta * V[0] / Rth) * np.sign(V[2]))+"    "+str(k * (V[0]/Rth + 2*velocity/dphi) * np.sign(delta)))
    return omega, v


class control_handle():

    def __init__(self):
        self.node       = rospy.init_node('controller', anonymous=True)
        self.dists_sub  = rospy.Subscriber('min_distances', twoFloats, self.msgCallback)
        self.tsub       = rospy.Subscriber('/clock', Clock, self.tCallback)
        self.vel_pub    = rospy.Publisher('cmd_vel', Twist, queue_size= 1000)
        self.vel        = Twist()
        self.velocity   = velocity
        self.vel.linear.x = self.velocity
        self.alpha_dot  = 0
        self.theta_dot  = 0
        self.min_dist   = dc
        self.min_del    = delc
        self.X_home     = np.empty(4)
        self.V_home     = np.empty(3)
        self.rate       = rospy.Rate(10) # 10hz
        self.engaged    = False
        self.time       = 0
        print("Starting controller")

    def run(self):

        t_start = self.time
        just_engaged = self.min_dist<dc
        just_free = not just_engaged
        while not rospy.is_shutdown():

            self.engaged = self.min_dist<dc and abs(self.min_del)<delc

            if np.linalg.norm(self.X_home[:2])<Rth:
                omega   = 0
                new_vel = 0

            elif not self.engaged:
                if just_free:
                    tf          = self.time - t_start
                    vf          = self.V_home[0]
                    just_free   = False
                omega, new_vel  = freeController(self.V_home, self.time-t_start, tf, vf)
                just_engaged    = True

            else:
                if just_engaged:
                    te          = self.time - t_start
                    ve          = self.V_home[0]
                    just_engaged = False
                omega, new_vel  = engagedController(self.V_home, self.min_del, self.time - t_start, te, ve)
                just_free       = True


            self.vel.linear.x   = new_vel
            self.vel.angular.z  = omega
            self.vel_pub.publish(self.vel)  

    def msgCallback(self, data):
        self.min_dist   = data.dist
        self.min_del    = data.angle
        self.X_home     = data.X
        self.velocity   = data.velocity
        self.alpha_dot  = data.alpha_dot
        self.theta_dot  = (self.velocity/Rth) * np.sin(self.X_home[2]-self.X_home[3])

        self.V_home = np.array([self.velocity, self.alpha_dot, self.theta_dot])

    def tCallback(self, data):
        self.time = data.clock.secs




if __name__ == '__main__':
    con_h = control_handle()
    try:
        con_h.run()
    except rospy.ROSInterruptException:
        pass