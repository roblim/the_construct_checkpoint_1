#! /usr/bin/env python
import rospy
from my_rb1_ros.srv import Rotate, RotateRequest, RotateResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from tf.transformations import euler_from_quaternion
from typing import Optional
from math import radians, pi, degrees as rad_to_degrees


class RotateServiceServer(object):
    
    # create messages that are used to publish response
    _response = RotateResponse()
    SUCCESS_STR = "Robot rotated successfully!"
    FAILURE_STR = "Robot rotation failed!"

    def __init__(self):
        self.rate = rospy.Rate(10)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_msg = Twist()
        self.odom_msg: Optional[Odometry] = None
        self.odom_sub = rospy.Subscriber(
            name="/odom",
            data_class=Odometry,
            callback=self.odom_callback
        )
        self._check_odom_ready()

        # creates the service
        self._service = rospy.Service('/rotate_robot', Rotate, self.callback)

        try:
            rospy.wait_for_service('/rotate_robot', timeout=5.0)
            rospy.loginfo("Service Ready")
        except rospy.ROSException:
            rospy.logerr("Service not available after 5 seconds")

    def _check_odom_ready(self):
        rospy.loginfo("Checking /odom")
        while self.odom_msg is None and not rospy.is_shutdown():
            try:
                self.odom_msg = rospy.wait_for_message("/odom", Odometry, timeout=1.0)
                rospy.logdebug("Current /odom READY=>" + str(self.odom_msg))
            except:
                rospy.logerr("Current /odom not ready yet, retrying...")
        rospy.loginfo("Checking /odom...DONE")

    def odom_callback(self, odom_msg: Odometry) -> None:
        self.odom_msg = odom_msg

    def get_yaw_angle_rad(self, normalize=False) -> Optional[float]:
        if not self.odom_msg:
            return None
        orientation: Quaternion = self.odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        
        if normalize:
            return self.normalize_rad_angle(rad_angle=yaw)
        else:
            return yaw

    def stop_robot(self) -> None:
        rospy.loginfo("Stopping robot...")
        self.move_msg = Twist()
        self.move_pub.publish(self.move_msg)

    def normalize_rad_angle(self, rad_angle: float) -> float:
        return (rad_angle + 2 * pi) % (2 * pi)

    def rotate_robot(self, degrees: int, angular_speed: float=0.5) -> None:
        rospy.loginfo("Service Requested")
        abs_rotation_rad: float = abs(radians(degrees))

        self.stop_robot()
        rospy.loginfo(f"Attempting to rotate robot by {degrees} degrees.")
        if degrees < 0:
            self.move_msg.angular.z = -1 * abs(angular_speed)
        elif degrees > 0:
            self.move_msg.angular.z = abs(angular_speed)

        rospy.loginfo(f"Normalized Initial Yaw: {rad_to_degrees(self.get_yaw_angle_rad(normalize=True)):.5f} degrees.")
        yaw_delta_abs = 0
        while yaw_delta_abs < abs_rotation_rad:
            yaw_i_rad = self.get_yaw_angle_rad(normalize=True)
            self.move_pub.publish(self.move_msg)
            self.rate.sleep()
            yaw_f_rad = self.get_yaw_angle_rad(normalize=True)

            yaw_delta = abs(yaw_f_rad - yaw_i_rad)
            # handle case at domain boundary
            if yaw_delta > pi:
                yaw_delta = 2 * pi - yaw_delta
            yaw_delta_abs += abs(yaw_delta)

            rospy.logdebug(f"Start yaw: {yaw_i_rad:7.5f}, Final yaw: {yaw_f_rad:7.5f}, yawD: {yaw_delta:7.5f}, Target absYawD: {abs_rotation_rad:7.5f}, absYawD: {yaw_delta_abs:7.5f}")

        self.stop_robot()
        self._response.result = self.SUCCESS_STR
        rospy.loginfo(self.SUCCESS_STR)
        rospy.loginfo(f"Normalized Final Yaw: {rad_to_degrees(self.get_yaw_angle_rad(normalize=True)):.5f} degrees.")

    def callback(self, request: RotateRequest) -> RotateResponse:
        try:
            self.rotate_robot(degrees=request.degrees, angular_speed=1) 
        except rospy.ROSException as e:
            self.stop_robot()
            self._response.result = self.FAILURE_STR
            rospy.logerr(self.FAILURE_STR)
            rospy.logerr(e)
        
        rospy.loginfo("Service Completed")
        return self._response


if __name__ == '__main__':
    rospy.init_node('rotate_service_server', log_level=rospy.INFO)
    service_server = RotateServiceServer()
    rospy.on_shutdown(service_server.stop_robot)
    rospy.spin()