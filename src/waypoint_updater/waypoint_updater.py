#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
## MR 27/11/2018:
# Int32 message type is used by /traffic_waypoint
# http://wiki.ros.org/rospy/Overview/Messages
from std_msgs.msg import Int32 #MR
# MR 28/11/2018:
# Inspired by Project Walkthrough video (Part 6: System Integration Lesson) - use of KDTree
# cKDTree implementation is C++
# https://stackoverflow.com/questions/6931209/difference-between-scipy-spatial-kdtree-and-scipy-spatial-ckdtree
# See directory file test_trees.py for comparison of performance with KDTree
from scipy.spatial import cKDTree #MR

import math
from math import sqrt

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # MR 27/11/2018:
        # /traffic_waypoint has message type std_msgs/Int32 
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb) #MR
        # MR 27/11/2018:
        # Cannot find publishing node for /obstacle_waypoint
        # Not sure if obstacle detection is a requirement:
        # https://carnd.slack.com/messages/C6NVDVAQ3/convo/C6NVDVAQ3-1503591353.000237/
        #rospy.Subscriber('/obstacle_waypoint', , self.obstacle_cb)
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # MR 27/11/2018:
        # Object parameters for inputs
        self.current_pose, self.track_map, self.track_length, self.track_cKDTree = None, None, None, None #MR
        # Set to true to print track co-ordinates to file
        self.do_print_track_map_2_file = False # MR
        # Object parameters for outputs
        self.final_waypoints = Lane() #MR
        # Initialise track index which is closest to the ego vehicle
        self.ego_idx = None #MR
		# Calculation parameters - publish rate is 50Hz
        self.publish_rate = rospy.Rate(50) #MR
        self.node_creation_time = rospy.Time.now().to_sec() #MR
		# Extracted from the distance function starter code
        self.dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)#MR
		
		# MR 27/11/2018:
		# rospy.spin() replaced with fixed rate publisher, as recommended in Project Walkthrough video (Part 6: System Integration Lesson)
        # Run while loop until Ctrl+C
        # Same approach as for running node as "Writing ROS Node - Lesson 5"
        while not rospy.is_shutdown(): #MR
            # If base_waypoints and current_pose have been received by the waypoint_loader node and simulator, 
            # then we have enough info to create final_waypoints
            # Same approach as Q & A Walkthrough videos for this project
            if self.track_map and self.current_pose: #MR
			    # If cKDTree has not yet been created, create the cKDTree
                if not self.track_cKDTree: #MR
                    self.create_tree() #MR
                # Calculate the final waypoints - Function is unfinished
                self.calculate_final_waypoints() #MR
                # Publish the final waypoints which have been calculated
                self.final_waypoints_pub.publish(self.final_waypoints) #MR
                # Print to logfiles. Publishing rate validated in logfile
                rospy.loginfo('Final Waypoints published by waypoint_updater node') #MR
                rospy.loginfo('Waypoint_updater node has been alive for %f seconds', self.get_alive_time()) #MR
            # Hold until 20ms has passed
            self.publish_rate.sleep() #MR

    # MR 27/11/2018:
    # Simple function which returns roughly how long the node has been alive.
    # Matches Writing ROS Node - Lesson 5 
    def get_alive_time(self): #MR
        return rospy.Time.now().to_sec() - self.node_creation_time #MR

    # MR 28/11/2018:
    # Print the map x and y track map points to a text file
    def send_map_2_file(self): #MR
        f = open("the_track_map_export.txt", "w") #MR
        for idx, waypoint in enumerate(self.track_map.waypoints): #MR
            f.write(''.join((str(waypoint.pose.pose.position.x), ',',str(waypoint.pose.pose.position.y), '\n'))) #MR
        f.close() #MR

    # MR 28/11/2018:
    # Function to create a cKDTree based on the imported track_map
    def create_tree(self): #MR
        # This function is called within rospy fixed rate while loop.
        # A prerequisite for calling the function is self.track_map != None.
        # Therefore, this function assumes that self.track_map has already been populated
        # Method below follows same approach as Project Walkthrough video (Part 6: System Integration Lesson)
        track_position_orientation = [[waypoint.pose.pose.position, waypoint.pose.pose.orientation] for waypoint in self.track_map.waypoints] #MR
        track_xy_tuple_array = [(position_orientation[0].x, position_orientation[0].y) for position_orientation in track_position_orientation] #MR
        # cKDTree built using list of (x,y) tuples
        self.track_cKDTree = cKDTree(track_xy_tuple_array) #MR

    def pose_cb(self, msg):
        # TODO: Implement
        # MR 27/11/2018:
        # Set the object pose parameter to the value of the ROS message
        # i.e. each time Simulator publishes a new position / orientation for the vehicle, update our stored values
        self.current_pose = msg #MR
        rospy.loginfo('Current Pose has been updated in waypoint_updater node') #MR
        #pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # MR 27/11/2018:
        # Base Waypoints are only sent once (see Lesson 5 notes on 'Waypoint Updater Node (Partial)').
        # Therefore, when this callback function runs store the track map points
        self.track_map = waypoints #MR
        # Number of points in the track map
        self.track_length = len(self.track_map.waypoints) #MR
        rospy.loginfo('Track Map has been stored in waypoint_updater node') #MR
        if self.do_print_track_map_2_file: #MR
            self.send_map_2_file() #MR
        # The following is just test code to check that interface with simulator is working:
        # First 200 track points are highlighted (may be some distance away from vehicle)
        # self.final_waypoints.waypoints = self.track_map.waypoints[0:LOOKAHEAD_WPS] #MR	
        #pass
		
    # MR 27/11/2018:
    # Function to set the final_waypoints 
    def calculate_final_waypoints(self):
        # Get the closest track point which is ahead of the ego vehicle
        self.get_closest_track_point_ahead() #MR
        # If it is possible to locate ego vehicle on track map
        if self.ego_idx: #MR
            ## Assign from the track map to the final waypoint vector 
            # Remove the previously sent final_waypoints
            self.final_waypoints.waypoints = [] #MR
            for idx in range(LOOKAHEAD_WPS): #MR
                # track_idx should handles wrap-around from end to beginning of the track
                # Note: This current approach only works for one driving direction
                track_idx = (idx + self.ego_idx) % self.track_length #MR
                # Set the final_waypoints, to be published in __init___ function
                self.final_waypoints.waypoints.append(self.track_map.waypoints[track_idx]) #MR

    def get_closest_track_point_ahead(self):
        # MR 28/11/2018
        use_distance_based = True	
		# This Function borrows method from Project Walkthough video. As per the video, closest point in space may not be in front of the vehicle
        # Find the index of the closest point in space, by querying the cKDTree
        (distance_to_closest, closest_idx_in_space) = self.track_cKDTree.query((self.current_pose.pose.position.x, self.current_pose.pose.position.y)) #MR
        previous_idx = self.track_length - 1 if closest_idx_in_space == 0 else closest_idx_in_space - 1 #MR
        next_idx = 0 if closest_idx_in_space == self.track_length - 1 else closest_idx_in_space + 1 #MR
        
        # Therefore, further processing is required to guaruntee that self.ego_idx is set to a point in front of vehicle
        # Note: Method assumes anticlockwise travel of the vehicle, as this is the direction that the track map vector runs
        closest_waypoint = self.track_map.waypoints[closest_idx_in_space] #MR
        previous_waypoint = self.track_map.waypoints[previous_idx] #MR
    
        # Distance from previous track point to ego vehicle position
        if (use_distance_based):
            d_prev_2_ego = self.dl(previous_waypoint.pose.pose.position, self.current_pose.pose.position) #MR
            d_prev_2_closest = self.dl(previous_waypoint.pose.pose.position , closest_waypoint.pose.pose.position) #MR
            d_closest_2_ego = self.dl(self.current_pose.pose.position , closest_waypoint.pose.pose.position) #MR
		
            # If the distance from previous point to ego vehicle is greater than the hypotenuse of right angled
		    # triangle which would correcpond with these lengths.. then vehicle is ahead of closest point
            is_ahead = True if d_prev_2_ego > sqrt( d_closest_2_ego**2 + d_prev_2_closest**2) else False #MR
        else:
		    # Dot product as demontsrated in Q & A video
			# Like quicker computation due to lack of sqrt
            v_closest_2_ego = [self.current_pose.pose.position.x - closest_waypoint.pose.pose.position.x, self.current_pose.pose.position.y - closest_waypoint.pose.pose.position.y] #MR
            v_closest_2_prev = [previous_waypoint.pose.pose.position.x - closest_waypoint.pose.pose.position.x, previous_waypoint.pose.pose.position.y - closest_waypoint.pose.pose.position.y] #MR
            dot = v_closest_2_ego[0] * v_closest_2_prev[0] + v_closest_2_ego[1] * v_closest_2_prev[1] #MR
            is_ahead = (dot < 0) #MR
		
        # Set the waypoint index of the ego vehicle
        self.ego_idx = next_idx if is_ahead else closest_idx_in_space #MR
        rospy.loginfo('Next Waypoint Index: %d', self.ego_idx) #MR
        
    
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        # MR 28/11/2018: Extracted lambda distance function. Added to __init__ as object parameter self.dl
        for i in range(wp1, wp2+1):
            dist += self.dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
