import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class SquareNode(Node):
    #Ez a fő rajzoló node
    def __init__(self):
        
        #kommunikációs csatornák létrehozása
        super().__init__('square_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        #Változók létrehozása
        self.state = 'moving'
        self.side_count = 0
        self.start_x = None
        self.start_y = None
        self.start_theta = None
        #Nem változtó paraméterek megadása
        self.declare_parameter('linear_speed', 1.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('target_distance', 3.0)

    def pose_callback(self, msg):
        current_pose = msg
        #Kezdő pozíció megadása
        if self.state == 'moving':
            if self.start_x is None:
                self.start_x = current_pose.x
                self.start_y = current_pose.y
                self.move_forward()
                return
            #Pithagorasszal utatozz távolság
            distance = math.hypot(current_pose.x - self.start_x, 
                                current_pose.y - self.start_y)
            target_dist = self.get_parameter('target_distance').value
            #Oda-e értünk check
            if distance >= target_dist:
                self.state = 'turning'
                self.start_theta = current_pose.theta #Fordulat elmentése
                self.turn()

        elif self.state == 'turning':
            #Checkolja hogy 90 fokot fordulunk-e
            delta_theta = self.normalize_angle(current_pose.theta - self.start_theta)
            if abs(delta_theta) >= math.pi/2:
                self.side_count += 1
                #Checkolja hogy kész-e a négyzet
                if self.side_count >= 4:
                    self.stop()
                    rclpy.shutdown()
                else:
                    #Következő oldali reset
                    self.state = 'moving'
                    self.start_x = current_pose.x
                    self.start_y = current_pose.y
                    self.move_forward()

    def move_forward(self):
        #Publish előre
        twist = Twist()
        twist.linear.x = self.get_parameter('linear_speed').value
        self.cmd_vel_pub.publish(twist)

    def turn(self):
        #Publish fordulás
        twist = Twist()
        twist.angular.z = self.get_parameter('angular_speed').value
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        #Minden mozgás stop
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    @staticmethod
    def normalize_angle(angle):
        #szög normalizálás
        return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    #Node vérgehajtás (ezzel szenvedtem a legtöbbet)
    rclpy.init(args=args)
    node = SquareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()