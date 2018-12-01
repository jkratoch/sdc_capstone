#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
# MR 29/11/2018
# Added TrafficLightArray below to attampt to test TL functionality
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
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
from math import sqrt #MR

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

# MR Added 29/11/2018
# For testing TL functionality before classifier created
# This is taken from TrafficLight.msg
UNKNOWN=4
GREEN=2
YELLOW=1
RED=0


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
		
        # Set to TRUE to test stopping at traffic lights, based on simulator's real reported
        # status for traffic lights
        self.do_test_TL = False #MR
        if (self.do_test_TL): #MR
		    # List of traffic light structures sent in topic /vehicle/traffic_lights
            self.tl_array = None #MR
			# List of Track Map indices corrsponding to the ytraffic lights in self.tl_array
            self.tl_idx_list = None #MR
			# Track Map index for the closest traffic light ahead of the ego vehicle
            self.closest_tl_en_idx = None #MR
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.tl_array_test_cb) #MR

        # TODO: Add other member variables you need below
        # MR 27/11/2018:
        # Object parameters for inputs
        self.current_pose, self.track_map, self.track_length, self.track_cKDTree, self.traffic_light_waypoint = None, None, None, None, None #MR
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
                #rospy.loginfo('Final Waypoints published by waypoint_updater node') #MR
                #rospy.loginfo('Waypoint_updater node has been alive for %f seconds', self.get_alive_time()) #MR
            # Hold until 20ms has passed
            self.publish_rate.sleep() #MR

    # MR 29/11/2018:
    # Callback to receive the traffic light array vector from the Simulator (purely for testing purposes before TL classifier is built)
    def tl_array_test_cb(self, msg): #MR
        self.tl_array = msg.lights #MR

    # MR 29/11/2018:
    # Test function to map traffic light array to track map waypoints
    def tl_array_processing_test(self): #MR
        # Query the cKDTree to find which Track Map index is closest to the traffic light locations
        # self.tl_idx_list is a list of the indices corresponding to the traffic lights
        self.tl_idx_list = [self.track_cKDTree.query((light.pose.pose.position.x, light.pose.pose.position.y))[1] 
		                    for light in self.tl_array] #MR
        #for idx in self.tl_idx_list: #MR
        #    rospy.loginfo('Traffic light found at track map index %d', idx) #MR
	
    # MR 29/11/2018
    # From Simulator TL data /vehicle/traffic_lights, find the closest TL ahead of the vehicle
    def get_next_tl_test(self):
        # Initialise local parameters
        track_idx_closest, list_enum_closest, diff_min_pos = -1, -1, 100000000 #MR
        for list_enum, track_idx in enumerate(self.tl_idx_list): #MR
            diff = track_idx - self.ego_idx #MR
            # If distance is positive but lower than previous
            if (diff > 0 and diff < diff_min_pos): #MR
                diff_min_pos = diff #MR
                track_idx_closest = track_idx #MR
                list_enum_closest = list_enum #MR
        if track_idx_closest == -1 or list_enum_closest == -1: #MR
            # If could now find a traffic light ahead, then take the first occuring traffic lis 
            track_idx_closest = min(self.tl_idx_list) #MR
            list_enum_closest = self.tl_idx_list.index(idx_closest) #MR
        # This is a tuple (x, y). x points to the element of closest TL in tl_idx_list. y points is the Track Map index for this TL
        self.closest_tl_en_idx = (list_enum_closest, track_idx_closest) #MR
		
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
        #rospy.loginfo('Current Pose has been updated in waypoint_updater node') #MR
        #pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # MR 27/11/2018:
        # Base Waypoints are only sent once (see Lesson 5 notes on 'Waypoint Updater Node (Partial)').
        # Therefore, when this callback function runs store the track map points
        #rospy.loginfo("OVERWRITTEN TRACK MAP")
        self.track_map = waypoints #MR
        # Number of points in the track map
        self.track_length = len(self.track_map.waypoints) #MR
        #rospy.loginfo('Track Map has been stored in waypoint_updater node') #MR
        if self.do_print_track_map_2_file: #MR
            self.send_map_2_file() #MR
        # The following is just test code to check that interface with simulator is working:
        # First 200 track points are highlighted (may be some distance away from vehicle)
        # self.final_waypoints.waypoints = self.track_map.waypoints[0:LOOKAHEAD_WPS] #MR	
        #pass
		
    # MR 27/11/2018:
    # Function to set the final_waypoints 
    def calculate_final_waypoints(self):
        # Delete the previously sent final_waypoints
        self.final_waypoints.waypoints = [] #MR
		
        # Get the closest track point which is ahead of the ego vehicle
        self.get_closest_track_point_ahead() #MR

        # Create a vector for final_waypoints with base speed component
        final_waypoints_base_speed = [] #MR
        # This is the list of Track Map indices which correspond to each of the final waypoints
        final_wp_idx_vect = [] #MR
        
        # If it is possible to locate ego vehicle on track map, then create a list of final_waypoints
		# Here, extract waypoints from Track Map, but do not correct the velocity yet
		# Method implemented for setting final waypoints borrows heavily from Project Walkthrough videos
        if self.ego_idx: #MR
            for idx in range(LOOKAHEAD_WPS): #MR
                # track_idx should handles wrap-around from end to beginning of the track
                # Note: This current approach only works for one driving direction
                track_idx = (idx + self.ego_idx) % self.track_length #MR
				# List of track waypoints corresponding to final waypoints
                final_wp_idx_vect.append(track_idx) #MR
                # Set the final_waypoints, to be published in __init___ function
                final_waypoints_base_speed.append(self.track_map.waypoints[track_idx]) #MR
        
		# This is the function which handles TL adjustments (speed)
        self.update_waypoint_velocity(final_waypoints_base_speed, final_wp_idx_vect) #MR
		
		
    # MR 01/12/2018:
	# Update the velocity of the final_waypoints, from the values given in the track map.
    def update_waypoint_velocity(self, final_waypoints_base_speed, final_wp_idx_vect):
		# MR 01/12/2018:
        # Set the velocity profile for the final waypoints
        # Currently, only run this code if testing without TL classifier implemented
		# Method implemented here for reducing speed borrows heavily from Project Walkthrough videos
		
        noRed = None
		
        # MR 29/11/2018
        # Only for testing TL functionality prior to developing classifier
        if self.do_test_TL: #MR
            if not self.tl_array:
                # Have not yet received info from Simulator to update the speed. Therefore don't update the speed.
                self.final_waypoints.waypoints = final_waypoints_base_speed #MR
                return #MR
            else:
                # If testing without classifier, update self.tl_idx_list based on /vehicle/traffic_lights
                self.tl_array_processing_test() #MR
                # Get the parameters for the next TL ahead
                self.get_next_tl_test() #MR
                (tl_en, tl_idx) = self.closest_tl_en_idx #MR
                stop_idx = max(0, tl_idx - 12) # MR - Want to stop 12 track points before the TL
		        # This is the next traffic light ahead of the vehicle
                tl = self.tl_array[tl_en]
                noRed = True if tl.state == GREEN else False
        else: 
			# MR 01/12/2018
            if not self.traffic_light_waypoint:
                self.final_waypoints.waypoints = final_waypoints_base_speed #MR
                return
            else:
                stop_idx = max(0, self.traffic_light_waypoint) # Check in simulation. 
                noRed = True if self.traffic_light_waypoint == -1 else False


        #rospy.loginfo('Car idx: %d, TL Closest: %d', self.ego_idx, tl_idx) #MR

        if noRed: #MR
			# If traffic light is green, drive at speed specified by Track Map
            self.final_waypoints.waypoints = final_waypoints_base_speed #MR
        else:
			# Method implemented here for reducing speed borrows heavily from Project Walkthrough videos
            #rospy.loginfo('Ego: %d, TL: %d ', self.ego_idx, stop_idx) #MR
            #ego_v = self.current_pose.twist.twist.linear.x
            # Total distance from the ego vehicle to the index which we want to stop at
            #d_total = self.distance(self.track_map, self.ego_idx, stop_idx)
            for it, waypoint in enumerate(final_waypoints_base_speed): #MR
			    W = Waypoint() #MR
			    W.pose = waypoint.pose #MR
			    v_init = self.get_waypoint_velocity(waypoint) #MR
			    wp_idx_on_track = final_wp_idx_vect[it] #MR
			    d = self.distance(self.track_map.waypoints, wp_idx_on_track, stop_idx) #MR
			    #v = max(0, min(ego_v, ego_v *(d)/d_total)) #MR
			    v = v_init * (d / 100) #MR -> SQRT not used. Linear decrease in velocity w/ distance is assumed. Max Decel is not accounted for.
			    v = 0 if v < 2 else v #MR - as per Project Walkthrough, if velocity low enough, just set target to 0 m/s
			    W.twist.twist.linear.x = min(v , v_init) #MR - base waypoints hold speed limit - therefore don't exceed this
			    self.final_waypoints.waypoints.append(W) #MR
			    #rospy.loginfo('V Target %d: D: %f', it + 1, d) #MR
			
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
        #rospy.loginfo('Next Waypoint Index: %d', self.ego_idx) #MR
        
    
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # MR 29/11/2018
        # rosmsg show std_msgs/Int32 = int32 data. Therefore, use data field to extract value
        # This will be -1 if not red light detected
        self.traffic_light_waypoint = msg.data #MR
        #rospy.loginfo('Traffic Light Waypoint is %d', self.traffic_light_waypoint) #MR
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
