import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import Executor, SingleThreadedExecutor

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Quaternion, Pose, PoseArray, PoseStamped, TransformStamped
from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster, TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_pose_stamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_inverse

import numpy as np
from numpy.random import random_sample, normal, choice
from random import random, sample
import math
import copy

from sklearn.neighbors import NearestNeighbors

def get_yaw_from_pose(p):
    """ Function that extracts yaw angle from given pose """

    return euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])[2]

def calculate_relative_rotation(pose_prev, pose_cur):
    """ Function that calculates the rotation transformation from between two poses, which is equal to (q2 * inverse(q1))"""

    q_1 = np.zeros(4)
    q_1[0] = pose_prev.orientation.x
    q_1[1] = pose_prev.orientation.y
    q_1[2] = pose_prev.orientation.z
    q_1[3] = -pose_prev.orientation.w
    
    q_2 = np.zeros(4)
    q_2[0] = pose_cur.orientation.x
    q_2[1] = pose_cur.orientation.y
    q_2[2] = pose_cur.orientation.z
    q_2[3] = pose_cur.orientation.w
    

    q_r = quaternion_multiply(q_2, q_1)

    return q_r

def apply_quaternion_transform(pose, q_r):
    """ Function that receives a given pose and desired quaternion transformation, and performs transformation on quaternion of given pose."""

    # Extracting pose quaternion:
    q_1 = np.zeros(4)
    q_1[0] = pose.orientation.x
    q_1[1] = pose.orientation.y
    q_1[2] = pose.orientation.z
    q_1[3] = pose.orientation.w

    # Calculating new quaternion
    q_2 = quaternion_multiply(q_r, q_1)
    
    # Creating new Quaternion object storing contents of q_2
    quart = Quaternion()
    quart.x = q_2[0]
    quart.y = q_2[1]
    quart.z = q_2[2]
    quart.w = q_2[3]
    
    return quart


def calculate_gaussian_prob(x, sigma):
    """ Function that calculates the value of a zero-centered Gaussian function with standard deviation given by sigma """ 

    c = 1.0 / (sigma * math.sqrt(2.0 * math.pi))
    prob = c * math.exp((-math.pow(x,2))/(2.0 * math.pow(sigma, 2)))
    return prob


class Particle:

    def __init__(self, pose, wt):
        self.pose = pose # Particle pose
        self.wt = wt # Particle weight

    def __str__(self):
        return ("Particle: [" + str(self.pose.position.x) + ", " + str(self.pose.position.y) + ", " + str(get_yaw_from_pose(self.pose)) + "]")


class MapClient(Node):
    
    def __init__(self):
        super().__init__('map_client')
        self.map_client = self.create_client(GetMap, '/map_server/map')

        # Wait for the service server to be available
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service server not available, waiting...')


    def get_map_data(self):
        request = GetMap.Request()
        future = self.map_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            return response.map
        else:
            self.get_logger().error("Failed to retrieve map data")
            return None


class ParticleFilter(Node):

    def __init__(self):
        super().__init__('particle_filter')

        self.particles_pub = self.create_publisher(PoseArray, "/particle_cloud", 10) # Publishes current particle cloud
        self.pose_pub = self.create_publisher(PoseStamped, "/pf_pose", 10) # Publishes robot's estimated pose calculated from particle filter
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.receiveScan, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value) # Receives laser scan messages from lidar
        self.odom_sub = self.create_subscription(Odometry, "/diff_cont/odom", self.receiveOdom, 10) # Receives odometry data
        
        # Set frame names:
        self.base_frame = "base_link"
        self.map_topic = "map"

        # Initializing variables:
        self.map = OccupancyGrid() # Occupancy grid        
        self.particle_cloud = [] # Particle cloud array
        self.pose_calc = Pose() # Estimated robot pose
        self.closest_distances = None
        
        # Defining parameters:
        self.declare_parameter("num_particles", 2000.0) # Number of particles
        self.declare_parameter("lin_thresh", 0.05) # Threshold value for linear movement before a particle filter update occurs
        self.declare_parameter("ang_thresh", 0.52359877559) # Threshold value for angular movement before a particle filter update occurs
        self.declare_parameter("motion_pos_noise_wt", 0.01) # Weight of noise function (in meters) added to the x- and y- positions in the particle motion model
        self.declare_parameter("motion_ang_noise_wt", 0.0) # Weight of noise function (in radians) added to the yaw angle in the particle motion model
        # self.declare_parameter("ang_meas_deg", [180, 225, 270, 315, 0, 45, 90, 135]) # List of desired angles to sample LiDAR range measurements from in degrees
        self.declare_parameter("z_max", 12.0) # Maximum scan range in meters
        self.declare_parameter("sigma_hit", 0.05) # Gaussian noise (standard deviation in meters) of the measurements 
        self.declare_parameter("z_hit", 0.85) # Weight of measurement noise distribution
        self.declare_parameter("resampling_pos_noise", 0.5) # Gaussian noise (standard deviation in meters) added to the x- and y- positions when resampling new particles
        
        # Retrieving parameters:
        self.num_particles = int(self.get_parameter('num_particles').get_parameter_value().double_value)
        self.lin_thresh = self.get_parameter('lin_thresh').get_parameter_value().double_value
        self.ang_thresh = self.get_parameter('ang_thresh').get_parameter_value().double_value
        self.motion_pos_noise_wt = self.get_parameter('motion_pos_noise_wt').get_parameter_value().double_value
        self.motion_ang_noise_wt = self.get_parameter('motion_ang_noise_wt').get_parameter_value().double_value
        # self.ang_meas_deg = self.get_parameter('ang_meas_deg').get_parameter_value().double_array_value
        self.z_max = self.get_parameter('z_max').get_parameter_value().double_value
        self.sigma_hit = self.get_parameter('sigma_hit').get_parameter_value().double_value
        self.z_hit = self.get_parameter('z_hit').get_parameter_value().double_value
        self.resampling_pos_noise = self.get_parameter('resampling_pos_noise').get_parameter_value().double_value

        # self.get_logger().info("%f" % (len(self.ang_meas_deg)))
        self.ang_meas_deg = [180, 225, 270, 315, 0, 45, 90, 135]

        self.odom_rec = None
        self.odom_pose_prev = None

        # Enable listening for and broadcasting coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.initialized = False


    def receiveOdom(self, data):
        """ This method receives odometry data. """
        
        self.odom_rec = data
        return


    def initialize_pf(self, data):
        """ This method initializes the particle filter. """

        self.map = data
        if self.map:

            # Coordinates of origin of map (lower left corner of the map) with respect to the map frame
            self.map_x = self.map.info.origin.position.x
            self.map_y = self.map.info.origin.position.y
    
            # Performing initializing operations:
            self.initialize_likelihood_field()
            self.initialize_particle_cloud(data.header)
            self.initialized = True            
            
        return



    def data_ind_from_grid_x_y(self, x_grid, y_grid):
        """ Method to calculate corresponding index in map data array using given grid x- and y-coordinates of the grid cell"""

        map_ind = (y_grid * self.map.info.width) + x_grid
        return map_ind

    def grid_x_y_from_data_ind(self, map_ind):
        """ Method to calculate corresponding row and column indices using given index in map data array
            NOTE: This assumes that (0, 0) corresponds to the lower left corner of the map
            Hence, the grid x-coordinate represents the column index, while the grid y-coordinate represents the row index """

        x_grid = float(map_ind % self.map.info.width)
        y_grid = float(math.floor(map_ind/self.map.info.width))
        
        return x_grid, y_grid

    def position_from_grid_x_y(self, x_grid, y_grid):
        # Method to calculate position x- and y- coordinates using given grid x- and y- coordinates
        # Specifically, this function places the calculated coordinates in the MIDDLE of the cell

        x = float((x_grid * self.map.info.resolution) + (self.map.info.resolution * 0.5) + self.map_x)
        y = float((y_grid * self.map.info.resolution) + (self.map.info.resolution * 0.5) + self.map_y)
        
        return x, y

    def grid_x_y_from_position(self, x, y):
        # Method to calculate the grid x- and y- coordinates of the map in which the given continuous x and y-coordinates lie

        x_grid = math.floor((x - self.map_x) / self.map.info.resolution) 
        y_grid = math.floor((y - self.map_y) / self.map.info.resolution) 

        return x_grid, y_grid

    def is_valid_cell_pos(self, x_grid, y_grid):
        """ Method that returns whether or not the grid cell coordinates correspond to a valid grid cell position within the map. """
        
        return (x_grid >= 0) and (y_grid >= 0) and (x_grid < self.map.info.width) and (y_grid < self.map.info.height)

  
    def initialize_particle_cloud(self, header):
        """ Method to initialize the particle cloud """

        # Retrieves list of indices of the map occupancy data array corresponding to empty cells:
        empty_cell_list = []

        for i in range(len(self.map.data)):
            if(self.map.data[i] == 0):
                empty_cell_list.append(i)

        empty_cell_list_rand = choice(empty_cell_list, self.num_particles).tolist()

        # Initializing particles:
        for i in range(self.num_particles):
            pose = Pose()

            # To initialize the positions, a random index corresponding to an empty cell will be retrieved:  
            #ind_rand = math.floor(list_len * random_sample())
            data_ind_rand = empty_cell_list_rand[i]
            (x_grid_rand, y_grid_rand) = self.grid_x_y_from_data_ind(data_ind_rand)
            (x_rand, y_rand) = self.position_from_grid_x_y(x_grid_rand, y_grid_rand)

            pose.position.x = x_rand
            pose.position.y = y_rand
            
            # To initialize the yaw rotation for each particle, a random angle between 0 and 2*PI will be generated:
            yaw_rand = (2 * np.pi) * random_sample()
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion_from_euler(0, 0, yaw_rand)

            wt = (1 / self.num_particles) # Initial particle weight
            part_new = Particle(pose, wt) # Initializing new particle
            self.particle_cloud.append(part_new)

        self.publish_particle_cloud(header)

        return



    def initialize_likelihood_field(self):
        """
        The purpose of initializing the likelihood field at the start of the filter is to 
        be able to quickly retrieve the distance between the nearest object and each
        calculated measured range measurement.
        """

        # Creates a 2-D numpy array to store all x- and y- coordinates of each grid cell of the map
        X = np.zeros((self.map.info.width*self.map.info.height, 2))

        # Placing each grid x- and y-coordinate of each grid cell into X, while also tracking the total number of occupied grid cells in the map:
        num_occupied = 0 # Number of occupied grid cells in the map
        for i in range(len(self.map.data)):
            X[i, 0], X[i, 1] = self.grid_x_y_from_data_ind(i) # Filling in each grid x- and y- coordinate 

            # If current cell is occupied, increment the counter:
            if self.map.data[i] > 0:
                num_occupied += 1
            elif self.map.data[i] != 1 and self.map.data[i] != 0:
                self.get_logger().info("%f" % (self.map.data[i]))


        # Creates a 2-D numpy array to store all x- and y- coordinates of each OCCUPIED grid cell of the map
        X_occup = np.zeros((num_occupied, 2))

        # Placing each grid x- and y- coordinate of each occupied grid cell into X_occup
        counter = 0
        for i in range(len(self.map.data)):
            if(self.map.data[i] > 0):
                x_grid, y_grid = self.grid_x_y_from_data_ind(i)
                X_occup[counter, 0] = x_grid
                X_occup[counter, 1] = y_grid                
                counter+=1


        # Use scikit learn's nearest neighbor algorithm:
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(X_occup)
        self.closest_distances, indices = nbrs.kneighbors(X)


        return


    def get_closest_obstacle_distance(self, x, y):
        """ Method to calculate the smallest distance to a registered obstacle for any given x- and y- coordinate """

        x_grid, y_grid = self.grid_x_y_from_position(x, y)
        is_valid = self.is_valid_cell_pos(x_grid, y_grid) # Checking to see if given x and y position corresponds to a valid position within the map

        if is_valid:
            map_ind = self.data_ind_from_grid_x_y(x_grid, y_grid)
            closest_dist = (self.closest_distances[map_ind][0] * self.map.info.resolution)
        else:
            return

        return closest_dist




    def receiveScan(self, data):
        """ This serves as the main method that runs the particle filter algorithm. """

        # Wait until the particle cloud is initialized:
        if not(self.initialized):
            return


        if not self.odom_rec:
            return
        
        self.odom_pose = self.odom_rec.pose.pose

        # If a previous odom pose doesn't exist, simply equate it to the current pose
        if not self.odom_pose_prev:
            self.odom_pose_prev = self.odom_pose
            return


        if self.particle_cloud:
            x_diff = self.odom_pose.position.x - self.odom_pose_prev.position.x
            y_diff = self.odom_pose.position.y - self.odom_pose_prev.position.y
            yaw_diff = get_yaw_from_pose(self.odom_pose) - get_yaw_from_pose(self.odom_pose_prev)

            # Performing particle update only if sufficient movement in either degree of freedom had occurred:
            if (np.abs(x_diff) > self.lin_thresh or np.abs(y_diff) > self.lin_thresh or np.abs(yaw_diff) > self.ang_thresh):

                # Updating particles with motion model:
                self.update_particles_with_motion_model()

                # Using the incoming LiDAR scans to update the particle weights:
                self.update_particle_weights_with_measurement_model(data)

                # Normalizing the particle weights:
                self.normalize_particles()

                # Resampling the particles based on their updated normalized weights:
                self.resample_particles()

                # Updating the estimated robot pose:
                self.update_estimated_robot_pose()

                # Publishing particle cloud and estimated robot pose:
                self.publish_particle_cloud(data.header)
                self.publish_estimated_robot_pose(data.header)

                # Updating odometry:
                self.odom_pose_prev = self.odom_pose

        return
    



    def update_particles_with_motion_model(self):
        # Method to calculate particles' expected poses using given motion model
        # Uniform noise is added to the particles' motion via the random() function and noise weights for position and orientation

        # Calculating displacements for each degree of freedom:
        x_diff = self.odom_pose.position.x - self.odom_pose_prev.position.x
        y_diff = self.odom_pose.position.y - self.odom_pose_prev.position.y
        q_r = calculate_relative_rotation(self.odom_pose_prev, self.odom_pose)


        # yaw_diff = get_yaw_from_pose(self.odom_pose) - get_yaw_from_pose(self.odom_pose_prev)
        # self.get_logger().info("Angle: %0.3f" % (get_yaw_from_pose(self.odom_pose)))
        # self.get_logger().info("Differences: %0.3f, %0.3f, %0.3f" % (x_diff, y_diff, yaw_diff))
        # self.get_logger().info(" ")
        
        for part in self.particle_cloud:

            # Updating positions with noise:
            part.pose.position.x += x_diff + ((self.motion_pos_noise_wt * random()) - (0.5 * self.motion_pos_noise_wt))
            part.pose.position.y += y_diff + ((self.motion_pos_noise_wt * random()) - (0.5 * self.motion_pos_noise_wt))
            part.pose.orientation = apply_quaternion_transform(part.pose, q_r)

            # Updating orientations with noise:
            part_yaw = get_yaw_from_pose(part.pose)
            part_yaw += ((self.motion_ang_noise_wt * random()) - (0.5 * self.motion_ang_noise_wt))
            
            # if(part_yaw > (2 * np.pi)):
            #     part_yaw -= (2 * np.pi)
            # elif(part_yaw < (-2 * np.pi)):
            #     part_yaw += (2 * np.pi)
            
            part.pose.orientation.x, part.pose.orientation.y, part.pose.orientation.z, part.pose.orientation.w = quaternion_from_euler(0, 0, part_yaw)
            

        return


    def update_particle_weights_with_measurement_model(self, data):
        """ Method to calculate particles' weights given current measurements """

        # ang_meas_deg = [180, 225, 270, 315, 0, 45, 90, 135] # List of desired angles to sample LiDAR range measurements from in degrees
        # ang_ind = [0, 45, 90, 135, 180, 225, 270, 315]
        ang_meas_rad = [math.radians(ang) for ang in self.ang_meas_deg] # Desired angles in radians
        ang_ind = [(ang - 180) + 360 if (ang - 180) < 0 else (ang - 180) for ang in self.ang_meas_deg] # Contains indices corresponding to relevant LiDAR range measurements in LaserScan data
        z = [data.ranges[ind] for ind in ang_ind] # Collecting relevant LiDAR measurements

        # Position of sensor relative to base_link:
        x_sens = 0.04001
        y_sens = 0

        # Measurement model parameters:        
        z_random = (1 - self.z_hit) # Weight of random measurements
        q = 1 # Weight of each particle

        # Using a likelihood field measurement model to calculate particle weights:
        for part in self.particle_cloud:

            
            # Extracting relevant pose data:
            x_part = part.pose.position.x
            y_part = part.pose.position.y
            ang_part = get_yaw_from_pose(part.pose)

            # Determine if current particle is within bounds. If not, then assign particle a small weight, and continue to next particle
            is_valid = self.get_closest_obstacle_distance(x_part, y_part)
            if not is_valid:
                part.wt = 0.0000000001
                continue

            # Iterating through each range measurement to calculate particle weight:
            for i in range(len(z)):
                if(float(z[i]) < float(self.z_max)):
                    x_z = x_part + (x_sens * np.cos(ang_part)) - (y_sens * np.sin(ang_part)) + (z[i] * np.cos(ang_part + ang_meas_rad[i]))
                    y_z = y_part + (y_sens * np.sin(ang_part)) - (x_sens * np.cos(ang_part)) + (z[i] * np.sin(ang_part + ang_meas_rad[i]))
                    dist = self.get_closest_obstacle_distance(x_z, y_z)
                    
                    # Evaluates whether or not the estimated LiDAR position is within the given map:
                    if dist:
                        q *= ((self.z_hit * calculate_gaussian_prob(dist, self.sigma_hit)) + (z_random/self.z_max))
                    else:
                        q = 0.001
                
                     

            part.wt = q # Assigning weight to particle
            q = 1 # Reset each particle weight value

        return



    def normalize_particles(self):
        """ Method to normalize particles' weights """

        # Calculating the total sum of the pre-normalized weights:
        wt_tot = sum(part.wt for part in self.particle_cloud)

        # Normalizing weights:
        for i in range(self.num_particles):
            self.particle_cloud[i].wt /= wt_tot
        
        return


    def resample_particles(self):
        """ Method to resample particles """

        wts = [part.wt for part in self.particle_cloud]
        wt_tot = sum(wts)
        sample = np.random.choice(list(range(self.num_particles)), self.num_particles, replace=True, p = wts)

        new_cloud = []
        # sigma = 0.5
        sigma = self.resampling_pos_noise

        # Extracting each particle
        for ind in sample:
            part = copy.deepcopy(self.particle_cloud[ind]) # Deep copying particle
            
            part.wt = (1/self.num_particles) # Assigning particle weight

            x_init = part.pose.position.x
            y_init = part.pose.position.y

            # Adding Gaussian noise to the x and y positions to each particle, and ensuring that new particle position is valid within the map
            is_valid_pos = False

            while(not is_valid_pos):
                part.pose.position.x = x_init + np.random.normal(0, sigma)
                part.pose.position.y = y_init + np.random.normal(0, sigma)
                x_grid, y_grid = self.grid_x_y_from_position(part.pose.position.x, part.pose.position.y)
                is_valid_pos = self.is_valid_cell_pos(x_grid, y_grid)

            new_cloud.append(part) # Adding each particle

        self.particle_cloud = new_cloud # Re-defining particle cloud


        return




    def update_estimated_robot_pose(self):
        """ Method to calculate and update the robot's pose estimate """

        x_avg = 0.0
        y_avg = 0.0
        yaw_avg = 0.0

        # Iterating through particle cloud to get total sum of x, y, and yaw values:
        for part in self.particle_cloud:
            x_avg += part.pose.position.x
            y_avg += part.pose.position.y
            yaw_avg += get_yaw_from_pose(part.pose)
        
        # Calculating average values:
        x_avg /= self.num_particles
        y_avg /= self.num_particles
        yaw_avg /= self.num_particles

        # Assigning average values to calculated robot pose:
        self.pose_calc.position.x = x_avg
        self.pose_calc.position.y = y_avg
        self.pose_calc.orientation.x, self.pose_calc.orientation.y, self.pose_calc.orientation.z, self.pose_calc.orientation.w = quaternion_from_euler(0, 0, yaw_avg)
            
        return


    def publish_particle_cloud(self, header):
        """ Method to publish the particle cloud """

        particle_cloud_pose_array = PoseArray()
        
        particle_cloud_pose_array.header = Header(stamp=header.stamp, frame_id=self.map_topic)
        particle_cloud_pose_array.poses = []

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        # Publishing particle cloud:
        self.particles_pub.publish(particle_cloud_pose_array)
        
        return



    def publish_estimated_robot_pose(self, header):
        """ Method to publish the estimated robot pose """

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.pose_calc
        
        robot_pose_estimate_stamped.header = Header(stamp=header.stamp, frame_id=self.map_topic)

        # Publishing pose:
        self.pose_pub.publish(robot_pose_estimate_stamped)

        # Attempting to publish estimated robot pose to map frame...
        t = TransformStamped()
        t.header.stamp = header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.pose_calc.position.x
        t.transform.translation.y = self.pose_calc.position.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = self.pose_calc.orientation.x
        t.transform.rotation.y = self.pose_calc.orientation.y
        t.transform.rotation.z = self.pose_calc.orientation.z
        t.transform.rotation.w = self.pose_calc.orientation.w

        self.tf_broadcaster.sendTransform(t)

        return





def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()

    map_client = MapClient()
    pf = ParticleFilter()

    map_data = map_client.get_map_data()
    
    if map_data is not None:
        pf.initialize_pf(map_data)
    
    executor.add_node(pf)
    executor.spin()

    # while rclpy.ok():
    #     rclpy.spin_once(pf)

    pf.destroy_node()
    rclpy.shutdown()