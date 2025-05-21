# #!/usr/bin/env python
# import rospy
# from std_msgs.msg import String

# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "Hello ROS %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass
#!/usr/bin/env python
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')  # Initialize the node with name 'talker'
        self.pub = self.create_publisher(String, 'chatter', 10)  # Create publisher
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz timer
        self.get_logger().info('Talker node started')

    def timer_callback(self):
        hello_str = String()
        hello_str.data = f"Hello ROS {self.get_clock().now().to_msg().sec}"
        self.get_logger().info(hello_str.data)
        self.pub.publish(hello_str)

def main():
    rclpy.init()  # Initialize ROS 2
    talker = Talker()  # Create instance of Talker node
    try:
        rclpy.spin(talker)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()  # Clean up the node
        rclpy.shutdown()  # Shutdown ROS 2

if __name__ == '__main__':
    main()