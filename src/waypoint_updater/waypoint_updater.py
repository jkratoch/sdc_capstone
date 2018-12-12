#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import cKDTree
import math
from math import sqrt


LOOKAHEAD_WPS = 200 # Number of waypoints to publish

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.current_pose = None
        self.track_map = None
        self.track_length = None
        self.track_cKDTree = None
        self.traffic_light_waypoint = -1
        self.final_waypoints = Lane()
        self.ego_idx = None
        self.node_creation_time = rospy.Time.now().to_sec()
        self.dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        
        # Set to true to print track co-ordinates to file
        self.do_print_track_map_2_file = False

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.track_map and self.current_pose:
                if not self.track_cKDTree:
                    self.create_tree()      
                    
                self.calculate_final_waypoints() 
                self.final_waypoints_pub.publish(self.final_waypoints) 

            rate.sleep()

    def get_alive_time(self):
        return rospy.Time.now().to_sec() - self.node_creation_time

    def send_map_2_file(self):
        f = open("the_track_map_export.txt", "w")
        for idx, waypoint in enumerate(self.track_map.waypoints):
            f.write(''.join((str(waypoint.pose.pose.position.x), ',',str(waypoint.pose.pose.position.y), '\n')))
        f.close()

    def create_tree(self):
        track_position_orientation = [[waypoint.pose.pose.position, waypoint.pose.pose.orientation] for waypoint in self.track_map.waypoints]
        track_xy_tuple_array = [(position_orientation[0].x, position_orientation[0].y) for position_orientation in track_position_orientation]
        self.track_cKDTree = cKDTree(track_xy_tuple_array)

    def pose_cb(self, msg):
        self.current_pose = msg

    def waypoints_cb(self, waypoints):
        self.track_map = waypoints
        self.track_length = len(self.track_map.waypoints)
        if self.do_print_track_map_2_file:
            self.send_map_2_file()

    def calculate_final_waypoints(self):
        self.final_waypoints.waypoints = []
        self.get_closest_track_point_ahead()
        final_waypoints_base_speed = []
        final_wp_idx_vect = []

        if self.ego_idx: 
            for idx in range(LOOKAHEAD_WPS):
                track_idx = (idx + self.ego_idx) % self.track_length 
                final_wp_idx_vect.append(track_idx)
                final_waypoints_base_speed.append(self.track_map.waypoints[track_idx])

        self.update_waypoint_velocity(final_waypoints_base_speed, final_wp_idx_vect)
        
    def update_waypoint_velocity(self, final_waypoints_base_speed, final_wp_idx_vect):
        if self.traffic_light_waypoint == -1:
            self.final_waypoints.waypoints = final_waypoints_base_speed
        else:
            stop_idx = max(0, self.traffic_light_waypoint)

            for it, waypoint in enumerate(final_waypoints_base_speed):
                W = Waypoint() 
                W.pose = waypoint.pose
                v_init = self.get_waypoint_velocity(waypoint)
                wp_idx_on_track = final_wp_idx_vect[it]
                d = self.distance(self.track_map.waypoints, wp_idx_on_track, stop_idx)
                #v = max(0, min(ego_v, ego_v *(d)/d_total)) #MR
                # v = v_init * (d / 100) #MR -> SQRT not used. Linear decrease in velocity w/ distance is assumed. Max Decel is not accounted for.
                v = sqrt(d)
                v = 0 if v < 2 else v
                W.twist.twist.linear.x = min(v , v_init)
                self.final_waypoints.waypoints.append(W)
            
    def get_closest_track_point_ahead(self):
        use_distance_based = True
        
        (distance_to_closest, closest_idx_in_space) = self.track_cKDTree.query((self.current_pose.pose.position.x, self.current_pose.pose.position.y))
        previous_idx = self.track_length - 1 if closest_idx_in_space == 0 else closest_idx_in_space - 1
        next_idx = 0 if closest_idx_in_space == self.track_length - 1 else closest_idx_in_space + 1
        closest_waypoint = self.track_map.waypoints[closest_idx_in_space]
        previous_waypoint = self.track_map.waypoints[previous_idx]
    
        if (use_distance_based):
            d_prev_2_ego = self.dl(previous_waypoint.pose.pose.position, self.current_pose.pose.position)
            d_prev_2_closest = self.dl(previous_waypoint.pose.pose.position , closest_waypoint.pose.pose.position)
            d_closest_2_ego = self.dl(self.current_pose.pose.position , closest_waypoint.pose.pose.position)
            is_ahead = True if d_prev_2_ego > sqrt( d_closest_2_ego**2 + d_prev_2_closest**2) else False
        else:
            v_closest_2_ego = [self.current_pose.pose.position.x - closest_waypoint.pose.pose.position.x, self.current_pose.pose.position.y - closest_waypoint.pose.pose.position.y]
            v_closest_2_prev = [previous_waypoint.pose.pose.position.x - closest_waypoint.pose.pose.position.x, previous_waypoint.pose.pose.position.y - closest_waypoint.pose.pose.position.y]
            dot = v_closest_2_ego[0] * v_closest_2_prev[0] + v_closest_2_ego[1] * v_closest_2_prev[1]
            is_ahead = (dot < 0)
        
        self.ego_idx = next_idx if is_ahead else closest_idx_in_space
        
    def traffic_cb(self, msg):
        self.traffic_light_waypoint = msg.data

    def obstacle_cb(self, msg):
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += self.dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
