#!/usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from controller.msg import twoFloats
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu


home_x, home_y = 0,0
dc = 10
number_of_bots = 3

class headquarters():
    def __init__(self, n):
        self.node = rospy.init_node('headquarters', anonymous=True)
        self.n = n

        self.bot_poses = [[0,[0,0,0,0],0,0] for i in range(self.n)]

        self.pubs = [rospy.Publisher(f'/tb3_{i+1}/min_distances', twoFloats, queue_size=1000) for i in range(self.n)]


    def run(self):\
    
        for i in range(self.n):

            rospy.Subscriber(f'/tb3_{i+1}/odom', Odometry, self.OdomCallback, i)
            rospy.Subscriber(f'/tb3_{i+1}/imu', Imu, self.ImuCallback, i)

        rospy.spin()
            


    def OdomCallback(self, data, i):

        pos = data.pose.pose
        position = pos.position

        # global x and y positions
        x, y = position.x, position.y
        #x and y positions relative to home
        x_rel = x - home_x
        y_rel = y - home_y

        #alpha relative to axis parallel to global x
        quaternion = pos.orientation
        _, __, alpha = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])         #alpha \in [-pi,pi]
        if alpha<0:
            alpha = 2*np.pi + alpha       #alpha \in [0,2pi]

        #theta relative to axis parallel to global x and origin as home
        theta = np.arctan2(y_rel,x_rel)
        if theta<0:
            theta = 2*np.pi + theta    #theta \in [0, 2pi]

        
        ### IMPORTANT: X stores x,y,alpha,theta of a bot
        X_home = np.array([x_rel, y_rel, alpha, theta])


        #velocity
        velocity = data.twist.twist.linear.x

        self.bot_poses[i][:3] = [i+1,X_home, velocity]
        min_data = get_min_data(self.bot_poses)

        for i in range(self.n):
            a = twoFloats()
            # print(min_data)
            a.dist = min_data[i][0]
            a.angle = min_data[i][1]
            a.X = self.bot_poses[i][1]
            a.velocity = (self.bot_poses[i][2])
            a.alpha_dot = self.bot_poses[i][3]
            self.pubs[i].publish(a)

    def ImuCallback(self, data, i):
        self.bot_poses[i][3] = (data.angular_velocity.x)




def get_min_data(bot_poses):
    result = [0]*number_of_bots

    for i in range(number_of_bots):

        my_bot_poses = bot_poses[i][1]
        min_dist = dc + 1
        min_angle = np.pi + 1

        for j in range(number_of_bots):

            test_bot_poses = bot_poses[j][1]

            if i!=j:

                rel_pos_vector = np.array(test_bot_poses[:2])-np.array(my_bot_poses[:2])
                dist = np.linalg.norm(rel_pos_vector)
                unit_rel_vector = rel_pos_vector/dist


                #angle of dij wrt global x axis
                angle_of_dij = np.arctan2(unit_rel_vector[1],unit_rel_vector[0])
                angle_of_dij = 2*np.pi*(angle_of_dij<0) + angle_of_dij

                #angle of heading direction of a bot wrt to global x axis
                angle_of_v = my_bot_poses[2]

                #angle of closest bot wrt to heading direction of first bot
                angle = angle_of_v - angle_of_dij

                if min_dist>dist:
                    min_dist = dist
                    min_angle = angle
                    
        result[i] = [min_dist, min_angle]

    return result


if __name__ == '__main__':
    manager = headquarters(number_of_bots)
    try:
        manager.run()
    except rospy.ROSInterruptException:
        pass