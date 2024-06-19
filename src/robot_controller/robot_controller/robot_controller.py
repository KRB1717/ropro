import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.target_position = [5.0, 5.0]  # 목표 위치
        self.current_position = [0.0, 0.0]  # 현재 위치 (여기서는 단순화)

    def timer_callback(self):
        twist = Twist()
        # 단순한 P 제어기 예제
        twist.linear.x = 0.5 * (self.target_position[0] - self.current_position[0])
        twist.angular.z = 0.5 * (self.target_position[1] - self.current_position[1])
        self.publisher.publish(twist)
        # 현재 위치 업데이트 (단순화)
        self.current_position[0] += twist.linear.x * 0.1
        self.current_position[1] += twist.angular.z * 0.1

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
