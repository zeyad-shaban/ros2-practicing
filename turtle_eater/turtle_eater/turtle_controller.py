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

# my programmer gut is telling me this node is doing way too much
class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller_node")
        self.get_logger().info(f"turtle_controller_node Started")
        
        self.declare_parameter("killer_name", "killer")
        self.declare_parameter("move_hz", 7)
        self.declare_parameter("Kp_v", 2.0)
        self.declare_parameter("Kp_w", 2.0)
        
        self.killer_name = self.get_parameter("killer_name").value
        self.move_hz: int = self.get_parameter("move_hz").value # type: ignore
        self.Kp_v = self.get_parameter('Kp_v').value
        self.Kp_w = self.get_parameter('Kp_w').value
        self.target = None
        
        self.vel = Vector3(x=0.0, y=0.0, z=0.0)
        self.angular_vel = Vector3(x=0.0, y=0.0, z=0.0)

        self.vel_publisher = self.create_publisher(Twist, f"{self.killer_name}/cmd_vel", 10)
        self.spawn_turtle_client = self.create_client(Spawn, "spawn_turtle")
        

        # killer seeing turtles
        self.alive_turtles_subscriber = self.create_subscription(AliveTurtles, "/alive_turtles", self.cb_alive_turtles, 10)
        
        # killer movement
        self.create_timer(1/self.move_hz, self.cb_move_killer)
        
        self.spawn_killer()
        
    def cb_move_killer(self):
        msg = Twist()
        msg.linear = self.vel
        msg.angular = self.angular_vel
        
        self.vel_publisher.publish(msg)
        
    def cb_alive_turtles(self, turtles: AliveTurtles):
        # kinda pointless how this would run whenenver something is published, whehter we have a target or not
        # there can be like lots of turtles that are there and we just cna't be aware of them as we are wiating for next tick there to run this...
        if self.target is not None:
            return
            
        killer: TurtleInfo | None = None
        for turtle in turtles.alive_turtles:
            turtle: TurtleInfo # i did this for vscode intellisense, not sure if there is a better way to do it
            if turtle.name == self.killer_name:
                killer = turtle
                break
        
        if killer == None:
            self.get_logger().error("Can't find teh killer turtle...")
            return
            
        closest_dist = float('inf')
        closest_turtle: TurtleInfo | None = None
        
        for turtle in turtles.alive_turtles:
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
        
        dx = closest_turtle.x - killer.x
        dy = closest_turtle.y - killer.y
        
        self.vel.x = self.Kp_v * (dx)
        self.vel.y = self.Kp_v * (dy)
        
        self.angular_vel.z = self.Kp_w * (math.atan2(dy, dx))
        
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