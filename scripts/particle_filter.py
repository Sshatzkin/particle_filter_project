#!/usr/bin/env python3

from hashlib import new
import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random, randrange, choices, uniform



def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(items, weights, n):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    # DONE
    return choices(items, weights=weights, k=n)


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 10

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        rospy.sleep(3)

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):

        self.map = data
    

    def initialize_particle_cloud(self):
        # DONE

        # Get map width and height
        map_width = self.map.info.width
        map_height = self.map.info.height
        resolution = self.map.info.resolution
        print("Map width: " + str(map_width))
        print("Map height: " + str(map_height))

        for i in range(self.num_particles):
            # create a new particle
            randPosition = Point(randrange(-map_width/8, map_width/4) * resolution, randrange(-map_height/8, map_height/4) * resolution, 0)
            randQuatvalues = quaternion_from_euler(0, 0, random() * 2 * math.pi)

            p = Particle(Pose(), 1.0)
            p.pose.position = randPosition
            p.pose.orientation = Quaternion(randQuatvalues[0], randQuatvalues[1], randQuatvalues[2], randQuatvalues[3])

            # add the particle to the particle cloud
            self.particle_cloud.append(p)

        
        self.normalize_particles()
        rospy.sleep(1)
        self.publish_particle_cloud()


    def normalize_particles(self):
        # DONE
        # make all the particle weights sum to 1.0

        total_weight = 0
        for i in range(self.num_particles):
            total_weight += self.particle_cloud[i].w
        print("Total Weight (Pre-norm): " + str(total_weight))
        for i in range(self.num_particles):
            self.particle_cloud[i].w = self.particle_cloud[i].w / total_weight

        check_total = 0
        for i in range(self.num_particles):
            check_total += self.particle_cloud[i].w
        print("Check Total: " + str(check_total))


    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

    
        for part in self.particle_cloud:
            print(part.pose.position)
            #print(get_yaw_from_pose(part.pose))
            #print(quaterniontoeurler(part.post.orientation))
            particle_cloud_pose_array.poses.append(part.pose)

        print("Publishing particle cloud of size: " + str(len(self.particle_cloud)))

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)
        print("Robot Estimate:",self.robot_estimate.position)


    def resample_particles(self):

        # DONE
        #Replace all of our particles with a probability proportional to the importance weights we've previously calculated. 

        new_particles = []
        weights = []
        for part in self.particle_cloud:
           weights.append(part.w)

        new_sample = draw_random_sample(self.particle_cloud, weights, self.num_particles)

        # Now, we add noise to the particles from this new sample
        noisy_particles = []
        for particle in new_sample:
            # add noise to the particle
            randPosition = Point(uniform(-0.1, 0.1) + particle.pose.position.x, uniform(-0.1, 0.1) + particle.pose.position.y, 0)
            randomYaw = uniform(-0.1, -1) * 2 *math.pi
            randQuatValues = quaternion_from_euler(0, 0, randomYaw)

            p = Particle(Pose(), 1.0)
            p.pose.position = randPosition
            p.pose.orientation = Quaternion(randQuatValues[0], randQuatValues[1], randQuatValues[2], randQuatValues[3])
            p.w = particle.w

            # add the particle to the particle cloud
            noisy_particles.append(p)
        
        
        self.particle_cloud = noisy_particles
        return


    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:
            print("Recieved scan and particle cloud active")
            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out
                print("Movement Threshold reached")
                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # TODO
        # based on the particles within the particle cloud, update the robot pose estimate
        average_point = Point(0,0,0)
        average_yaw = 0
        for part in self.particle_cloud:
            average_point.x += part.pose.position.x
            average_point.y += part.pose.position.y
            average_yaw += get_yaw_from_pose(part.pose)
        average_point.x /= len(self.particle_cloud)
        average_point.y /= len(self.particle_cloud)
        average_yaw /= len(self.particle_cloud)

        print("Estimated Robot Pose:",average_point, " Yaw: ", average_yaw)   
        self.robot_estimate.position = average_point
        quatVals = quaternion_from_euler(0, 0, average_yaw)
        self.robot_estimate.orientation = Quaternion(quatVals[0], quatVals[1], quatVals[2], quatVals[3])
        return

    
    def update_particle_weights_with_measurement_model(self, data):

        # TODO
        #Compare actual sensor readings w/ particle's "readings", and assign a weight based on difference

        #max_x = self.map.info.width * self.map.info.resolution
        #max_y = self.map.info.height * self.map.info.resolution

        """
        for i in range(self.num_particles):

            #Need to fill in these particle locations
            particle_dist_x = max_x - self.particle_cloud[i].position.x 
            particle_dist_y = None 
            particle_dist_z = None

            #Is laser_pose the right function to get sensor readings? CHECK
            self.particle_cloud[i].w = np.abs(self.laser_pose.pose.position.x - particle_dist_x) 
            +  np.abs(self.laser_pose.pose.position.y - particle_dist_y) 
            +  np.abs(self.laser_pose.pose.position.z - particle_dist_z)

            self.particle_cloud[i].w = np.reciprocal(self.particle_cloud[i].w)
            
                                    
        """


        return

        

    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # TODO
        """
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        """
        """
        #Might need to invert calculations for yaw or something? DOUBLE CHECK
        delta_x = curr_x - old_x
        delta_y = curr_y - old_y
        delta_yaw = get_yaw_from_pose(self.odom_pose.pose) - get_yaw_from_pose(self.odom_pose_last_motion_update.pose)


        #Is our yaw in here z? DOUBLE CHECK
        #Probably need to get it from Quaternion
        for i in range(self.num_particles):
            self.particle_cloud[i].position.x += delta_x
            self.particle_cloud[i].position.y += delta_y
            self.particle_cloud[i].position.z += delta_yaw


        """
        return 


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









