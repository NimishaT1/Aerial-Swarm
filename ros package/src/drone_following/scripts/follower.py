#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geographiclib.geodesic import Geodesic
from geographic_msgs.msg import GeoPoseStamped

# Offsets in meters
OFFSET_BACK = 2.0
OFFSET_UP = 1.0
SAFE_ALT_BUFFER = 5.0  # Always command at least 5m above follower

# Global variables
leader_gps = None
leader_state = State()
follower_state = State()
follower_gps = None
setpoint_started = False

# Callbacks
def leader_gps_cb(msg):
    global leader_gps
    leader_gps = msg

def leader_state_cb(msg):
    global leader_state
    leader_state = msg

def follower_state_cb(msg):
    global follower_state
    follower_state = msg

def follower_gps_cb(msg):
    global follower_gps
    follower_gps = msg

# Compute offset lat/lon
def offset_gps_position(lat, lon, offset_north, offset_east):
    result = Geodesic.WGS84.Direct(lat, lon, 0, offset_north)
    temp_lat, temp_lon = result['lat2'], result['lon2']
    result2 = Geodesic.WGS84.Direct(temp_lat, temp_lon, 90, offset_east)
    return result2['lat2'], result2['lon2']

# Build GeoPoseStamped message
def build_target(lat, lon, alt):
    msg = GeoPoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.latitude = lat
    msg.pose.position.longitude = lon
    msg.pose.position.altitude = alt
    return msg

def main():
    global setpoint_started
    rospy.init_node('geo_follower')

    # Subscribers
    rospy.Subscriber('/drone_a/mavros/global_position/global', NavSatFix, leader_gps_cb)
    rospy.Subscriber('/drone_a/mavros/state', State, leader_state_cb)
    rospy.Subscriber('/drone_b/mavros/state', State, follower_state_cb)
    rospy.Subscriber('/drone_b/mavros/global_position/global', NavSatFix, follower_gps_cb)

    # Publisher
    pub = rospy.Publisher('/drone_b/mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)

    # Services
    rospy.wait_for_service('/drone_b/mavros/cmd/arming')
    rospy.wait_for_service('/drone_b/mavros/set_mode')
    arming_srv = rospy.ServiceProxy('/drone_b/mavros/cmd/arming', CommandBool)
    mode_srv = rospy.ServiceProxy('/drone_b/mavros/set_mode', SetMode)

    rate = rospy.Rate(5)

    rospy.loginfo("Waiting for leader GPS...")
    while leader_gps is None and not rospy.is_shutdown():
        rate.sleep()

    rospy.loginfo("Leader GPS received. Waiting for Drone A to arm...")
    while not leader_state.armed and not rospy.is_shutdown():
        rospy.logwarn_throttle(5, "Drone A not armed yet...")
        rate.sleep()

    rospy.loginfo("Drone A armed. Pre-streaming dummy setpoints...")

    dummy_alt = leader_gps.altitude + OFFSET_UP + SAFE_ALT_BUFFER
    dummy_target = build_target(leader_gps.latitude, leader_gps.longitude, dummy_alt)
    for _ in range(100):
        pub.publish(dummy_target)
        rate.sleep()

    # Set GUIDED mode
    rospy.loginfo("Setting Drone B to GUIDED mode...")
    try:
        mode_resp = mode_srv(0, "GUIDED")
        if mode_resp.mode_sent:
            rospy.loginfo("GUIDED mode set successfully.")
        else:
            rospy.logwarn("GUIDED mode rejected.")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to set GUIDED mode: %s", e)

    # Arm Drone B
    rospy.loginfo("Arming Drone B...")
    try:
        arm_resp = arming_srv(True)
        if arm_resp.success:
            rospy.loginfo("Drone B armed successfully.")
        else:
            rospy.logwarn("Drone B arming failed.")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to arm Drone B: %s", e)

    rospy.loginfo("Starting GPS-following loop...")

    while not rospy.is_shutdown():
        if leader_gps is None:
            rospy.logwarn("No leader GPS fix.")
            rate.sleep()
            continue

        # Safety disarm
        if not leader_state.armed and follower_state.armed:
            rospy.logwarn("Drone A disarmed! Disarming Drone B...")
            try:
                arming_srv(False)
                rospy.loginfo("Drone B disarmed.")
            except rospy.ServiceException as e:
                rospy.logerr("Disarm failed: %s", e)
            setpoint_started = False
            continue

        if not follower_state.armed:
            rospy.logwarn_throttle(5, "Drone B not armed. Waiting...")
            rate.sleep()
            continue

        if follower_state.mode != "GUIDED":
            rospy.logwarn_throttle(5, "Drone B not in GUIDED mode.")
            rate.sleep()
            continue

        # Calculate target position
        offset_lat, offset_lon = offset_gps_position(
            leader_gps.latitude,
            leader_gps.longitude,
            -OFFSET_BACK,
            0
        )

        if follower_gps:
            target_alt = max(follower_gps.altitude + SAFE_ALT_BUFFER, leader_gps.altitude + OFFSET_UP)
        else:
            target_alt = leader_gps.altitude + OFFSET_UP + SAFE_ALT_BUFFER

        target = build_target(offset_lat, offset_lon, target_alt)
        pub.publish(target)

        if not setpoint_started:
            rospy.loginfo("Setpoint publishing started — Drone B is now following Drone A.")
            setpoint_started = True

        rospy.loginfo_throttle(2, f"Setpoint: lat={offset_lat:.6f}, lon={offset_lon:.6f}, alt={target_alt:.2f}")
        rate.sleep()

if __name__ == "__main__":
    main()
