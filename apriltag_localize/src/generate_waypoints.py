import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math 

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.subscription = self.create_subscription(
            PoseStamped,
            'tag_detections/tagpose_inertial',
            self.listener_callback,
            10)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_local_position = VehicleLocalPosition()

        self.publisher = self.create_publisher(PoseStamped, 'waypoint', 10)
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        
        self.poses = {0: None,1:None,2:None}
        
        self.current_waypoint = 0
        self.queue_x = [900,56,1000,4555,1,900,56,1000,4555,1,900,56,1000,4555,1,900,56,1000,4555,900,56,1000,4555,1,900,56,1000,4555,900,56,1000,4555,1,900,56,1000,4555,1]
        self.queue_y = [900,56,1000,4555,1,900,56,1000,4555,1,900,56,1000,4555,1,900,56,1000,4555,900,56,1000,4555,1,900,56,1000,4555,900,56,1000,4555,1,900,56,1000,4555,1]

        self.timer = self.create_timer(1.0, self.publish_waypoint)


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        #self.get_logger().info(f"Current position {[vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z]}")



    def listener_callback(self, msg):
        if not msg.header.frame_id == '':
            frame_id = int(msg.header.frame_id)
            if frame_id in self.poses:
                if self.poses[frame_id] == None:
                    x = round(msg.pose.position.x,1)
                    y = round(msg.pose.position.y,1)

                    mean_x = sum(self.queue_x)/len(self.queue_x)

                    mean_y = sum(self.queue_y)/len(self.queue_y)

                    if abs(mean_x - x )<0.1 and abs(mean_y -y)<0.1:
                        self.poses[frame_id] = msg
                        self.queue_x = [900,56,1000,4555,1,900,56,1000,4555,1,900,56,1000,4555,1,900,56,1000,4555,900,56,1000,4555,1,900,56,1000,4555,900,56,1000,4555,1,900,56,1000,4555,1]
                        self.queue_y = [900,56,1000,4555,1,900,56,1000,4555,1,900,56,1000,4555,1,900,56,1000,4555,900,56,1000,4555,1,900,56,1000,4555,900,56,1000,4555,1,900,56,1000,4555,1]
                        self.get_logger().info(f'Recorded apriltag- {frame_id}')

                    else:
                        self.queue_x.pop(0)
                        self.queue_x.append (x)  
                        self.queue_y.pop(0)
                        self.queue_y.append (y)                      
                    #self.get_logger().info(f'Received pose for object {frame_id}')
                    #self.get_logger().info(f"WP position {[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]}")


    def publish_waypoint(self):

        if self.current_waypoint in self.poses:
            publish_wp =True
            for id in self.poses:
                if self.poses[id] == None:
                    publish_wp = False
            
            if publish_wp == True:

                waypoint_pose = self.poses[self.current_waypoint]

                if not waypoint_pose == None:
                    distance =    math.sqrt((waypoint_pose.pose.position.x - self.vehicle_local_position.x )**2 + (waypoint_pose.pose.position.y - self.vehicle_local_position.y )**2)

                    if distance>0.2:
                        msg = PoseStamped()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = waypoint_pose.header.frame_id
                        
                        # Example pose data
                        msg.pose.position.x = waypoint_pose.pose.position.x
                        msg.pose.position.y = waypoint_pose.pose.position.y
                        msg.pose.position.z = waypoint_pose.pose.position.z
                        
                        msg.pose.orientation.x = 0.0
                        msg.pose.orientation.y = 0.0
                        msg.pose.orientation.z = 0.0
                        msg.pose.orientation.w = 1.0

                        self.publisher.publish(msg)
                        self.get_logger().info(f"Waypoint published {[waypoint_pose.header.frame_id]}")

                    else:
                        self.current_waypoint+=1
                        if self.current_waypoint ==3:
                            self.current_waypoint=0

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
