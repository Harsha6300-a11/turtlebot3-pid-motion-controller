#PID controller

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math


class Turtlebot3controller :
    def __init__(self):
        rospy.init_node("Turtlebot3_controller", anonymous =True)
        #initial paramaters
        self.x = 0
        self.y = 0
        self.yaw= 0
        
        #final parametrs
        self.x_goal = 5.0
        self.y_goal = 5.0

        #defining final distance parametrs as zero
        self.last_distance_error = 0
        self.sum_distance_error = 0

        #defining final angle paramters as zero
        self.last_angle_error = 0
        self.sum_angle_error = 0
        
        #linear velocity
        self.kp_linear = 0.2
        self.ki_linear = 0.3
        self.kd_linear = -0.2

        #angular velocity
        self.kp_angular = 0.2
        self.ki_angular = 0.1
        self.kd_angular = -0.2

        self.distance_tolerance = 0.2
        
        rospy.subscriber('/odom',Odometry,self.Odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)
        self.rate= rospy.Rate(10)
        
    def Odom_callback(self,msgs):
        
        self.x = msgs.pose.pose.position.x
        self.y = msgs.pose.pose.position.y
        orientation = msgs.pose.pose.orientation
        orientation_q = [orientation.x ,orientation.y ,orientation.z ,orientation.w]
        (_,_,self.yaw) = euler_from_quaternion(orientation_q)
        
    def compute_control(self):
        
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        distance_error = math.sqrt( dx**2 + dy**2 )

        angle_to_goal = math.atan2(dy,dx)
        angle_error = angle_to_goal - self.yaw
        angle_error = math.atan2(math.sin(angle_error),math.cos(angle_error))

        dt = 1.0 /10
        
        #linear
        
        self.sum_distance_error += distance_error * dt
        d_derivative_error = ( distance_error - self.last_distance_error ) / dt
       

        linear_speed = ( self.kp_linear * distance_error ) + (self.ki_linear * self.sum_distance_error)+( self.kd_linear * d_derivative_error)

        self.last_distance_error = distance_error
         
        #angular
         
        self.sum_angle_error += angle_error * dt
        d_angle_error = (angle_error - self.last_angle_error ) / dt
        

        angular_speed =  (self.kp_angular * angle_error  ) +( self.kd_angular * d_angle_error ) +( self.ki_angular * self.sum_angle_error)

        self.last_angle_error = angle_error

        if(distance_error > self.distance_tolerance ):
            return 0.0, 0.0

        return linear_speed , angular_speed
    
    def run(self):
        while not rospy.is_shutdown():
            v, w = self.compute_control()
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            self.cmd_vel_pub.publish (twist)
            self.rate.sleep()   

if __name__=='__main__':
    try:
        controller = Turtlebot3controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass 
        
