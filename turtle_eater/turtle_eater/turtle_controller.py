#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill
import random
import math
from turtlesim.srv import Spawn
from turtle_eater_interface.msg import TurtleInfo, AliveTurtles
from geometry_msgs.msg import Vector3

def normalize_angle(a):
    # map to [-pi, pi)
    return (a + math.pi) % (2*math.pi) - math.pi

# my programmer gut is telling me this node is doing way too much
class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller_node")
        self.get_logger().info(f"turtle_controller_node Started")
        
        self.declare_parameter("killer_name", "killer")
        self.declare_parameter("move_hz", 7)
        self.declare_parameter("Kp_v", 1.0)
        self.declare_parameter("Kp_w", 1.0)
        self.declare_parameter("kill_dist_thresh", 0.7)
        
        self.killer_name: str = self.get_parameter("killer_name").value # type: ignore
        self.move_hz: int = self.get_parameter("move_hz").value # type: ignore
        self.Kp_v: float = self.get_parameter('Kp_v').value # type: ignore
        self.Kp_w: float = self.get_parameter('Kp_w').value # type: ignore
        self.kill_dist_thresh: float = self.get_parameter('kill_dist_thresh').value # type: ignore
        self.target = None
        self.killer = None
        self.alive_turtles: list[TurtleInfo] = []
        
        self.vel = Vector3(x=0.0, y=0.0, z=0.0)
        self.angular_vel = Vector3(x=0.0, y=0.0, z=0.0)

        self.vel_publisher = self.create_publisher(Twist, f"{self.killer_name}/cmd_vel", 10)
        self.spawn_turtle_client = self.create_client(Spawn, "spawn_turtle")
        

        # killer seeing turtles
        self.create_timer(1/10, self.cb_set_target_turtle)
        self.alive_turtles_subscriber = self.create_subscription(AliveTurtles, "/alive_turtles", self.cb_set_alive_turtles, 10)
        self.catch_turtle_call = self.create_client(Kill, "/catch_turtle")
        
        # killer movement
        self.create_timer(1/self.move_hz, self.cb_move_killer)
        
        self.spawn_killer()
        
    def cb_move_killer(self):
        if self.target is None or self.killer is None:
            return
            
        self.get_logger().info(f"killer pos: {self.killer.x}, {self.killer.y}")

        msg = Twist()
        msg.linear = self.vel
        msg.angular = self.angular_vel
        
        self.vel_publisher.publish(msg)
        
        dx = self.target.x - self.killer.x
        dy = self.target.y - self.killer.y
        dist = math.sqrt(dy**2 + dx**2)
        
        if dist <= self.kill_dist_thresh:
            msg = Kill.Request()
            msg.name = self.target.name
            future = self.catch_turtle_call.call_async(msg)
            self.target = None
            
        
    def cb_set_alive_turtles(self, turtles: AliveTurtles):
        self.alive_turtles = turtles.alive_turtles
        
    def cb_set_target_turtle(self):
        killer: TurtleInfo | None = None
        for turtle in self.alive_turtles:
            turtle: TurtleInfo # i did this for vscode intellisense, not sure if there is a better way to do it
            if turtle.name == self.killer_name:
                killer = turtle
                break
        
        if killer == None:
            self.get_logger().error("Can't find teh killer turtle...")
            return
            
        closest_dist = float('inf')
        closest_turtle: TurtleInfo | None = None
        
        for turtle in self.alive_turtles:
            turtle: TurtleInfo
            if turtle.name == self.killer_name:
                continue
            
            dist = math.sqrt((turtle.x - killer.x)**2 + (turtle.y - killer.y)**2)
            if dist < closest_dist:
                closest_turtle = turtle
                closest_dist = dist
                
        if closest_turtle is None:
            self.vel = Vector3(x=0.0, y=0.0, z=0.0)
            self.angular_vel = Vector3(x=0.0, y=0.0, z=0.0)
            return
            
        # Calculation to target
        self.target = closest_turtle
        self.killer = killer
        
        dx = closest_turtle.x - killer.x
        dy = closest_turtle.y - killer.y
        
        dist = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        yaw_err = (desired - killer.theta + math.pi) % (2*math.pi) - math.pi
        
        linear  = self.Kp_v * dist * math.cos(yaw_err)   # or use: Kp_v * dist
        angular = self.Kp_w * yaw_err
        
        self.vel.x = linear
        self.angular_vel.z = angular
        
    def spawn_killer(self):
        req = Spawn.Request()
        req.name = self.killer_name
        req.x = float(random.randint(3, 8))
        req.y = float(random.randint(3, 8))
        req.theta = random.uniform(0, 2 * math.pi)
        
        while not self.spawn_turtle_client.wait_for_service(1):
            self.get_logger().warning("Waiting for spawn turtle client...")
            
        fut = self.spawn_turtle_client.call_async(req)
        fut.add_done_callback(self.cb_hunter_spawned)
        
    def cb_hunter_spawned(self, future):
        res: Spawn.Response = future.result()
        if res.name == "":
            self.get_logger().error("Spawning failed, probably hunter name already taken")
            return

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()