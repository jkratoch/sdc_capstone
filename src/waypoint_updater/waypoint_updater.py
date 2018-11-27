#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
## MR 27/11/2018:
# Int32 message type is used by /traffic_waypoint
# http://wiki.ros.org/rospy/Overview/Messages
from std_msgs.msg import Int32

import math

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
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # MR 27/11/2018:
        # Cannot find publishing node for /obstacle_waypoint
        # Not sure if obstacle detection is a requirement:
        # https://carnd.slack.com/messages/C6NVDVAQ3/convo/C6NVDVAQ3-1503591353.000237/
        #rospy.Subscriber('/obstacle_waypoint', , self.obstacle_cb)
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # MR 27/11/2018:
        # Object parameters for inputs
        self.current_pose, self.track_map, self.track_length = None, None, None
        # Object parameters for outputs
        self.final_waypoints = Lane()
        # Initialise track index which is closest to the ego vehicle
        self.ego_idx = None
		# Calculation parameters - publish rate is 50Hz
        self.publish_rate = rospy.Rate(50)
        self.node_creation_time = rospy.Time.now().to_sec()
		
		# MR 27/11/2018:
		# rospy.spin() replaced with fixed rate publisher, as recommended in project Q & A video
        # Run while loop until Ctrl+C
        # Same approach as for running node as "Writing ROS Node - Lesson 5"
        while not rospy.is_shutdown():
            # If base_waypoints and current_pose have been received by the waypoint_loader node and simulator, 
            # then we have enough info to create final_waypoints
            # Same approach as Q & A Walkthrough videos for this project
            if self.track_map and self.current_pose:
                # Calculate the final waypoints - Function is unfinished
                #self.calculate_final_waypoints()
                # Publish the final waypoints which have been calculated
                self.final_waypoints_pub.publish(self.final_waypoints)
                # Print to logfiles. Publishing rate validated in logfile
                rospy.loginfo('Final Waypoints published by waypoint_updater node')
                rospy.loginfo('Waypoint_updater node has been alive for %f seconds', self.get_alive_time())
            # Hold until 20ms has passed
            self.publish_rate.sleep()

    # MR 27/11/2018:
    # Simple function which returns roughly how long the node has been alive.
    # Matches Writing ROS Node - Lesson 5 
    def get_alive_time(self):
        return rospy.Time.now().to_sec() - self.node_creation_time

    def pose_cb(self, msg):
        # TODO: Implement
        # MR 27/11/2018:
        # Set the object pose parameter to the value of the ROS message
        # i.e. each time Simulator publishes a new position / orientation for the vehicle, update our stored values
        self.current_pose = msg
        rospy.loginfo('Current Pose has been updated in waypoint_updater node')
        #pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # MR 27/11/2018:
        # Base Waypoints are only sent once (see Lesson 5 notes on 'Waypoint Updater Node (Partial)').
        # Therefore, when this callback function runs store the track map points
        self.track_map = waypoints
        # Number of points in the track map
        self.track_length = len(self.track_map.waypoints)
        rospy.loginfo('Track Map has been stored in waypoint_updater node')
        # The following is just test code to check that interface with simulator is working:
        # First 200 track points are highlighted (may be some distance away from vehicle)
        self.final_waypoints.waypoints = self.track_map.waypoints[0:LOOKAHEAD_WPS]			
        #pass
		
    # MR 27/11/2018:
    # Function to set the final_waypoints 
    def calculate_final_waypoints(self):
        # Get the closest track point which is ahead of the ego vehicle
        self.ego_idx = self.get_closest_track_point_ahead()
        # If it is possible to locate ego vehicle on track map
        if self.ego_idx:
            ## Assign from the track map to the final waypoint vector 
            # Iterate idx from 0 to 199
            for idx in range(LOOKAHEAD_WPS):
                # track_idx should handles wrap-around from end to beginning of the track
                # Note: This current approach only works for one driving direction
                track_idx = (idx + self.ego_idx) % self.track_length
                # Set the final_waypoints, to be published in __init___ function
                self.final_waypoints.waypoints[idx] = self.track_map[track_idx]

    def get_closest_track_point_ahead(self):
        # MR 27/11/2018 
        # Unfinished...
        pass
    
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
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
