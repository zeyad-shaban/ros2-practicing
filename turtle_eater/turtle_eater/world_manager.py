#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import uuid
import random
import math
from turtlesim.srv import Spawn, Kill
from turtle_eater_interface.msg import AliveTurtles, TurtleInfo

# teh code is getting quite hard to manage, stuff are mixed together, cant' tell which function is internal which is publisher which is service :( i'm not following any conventiosn for what named what an dhow 
# the fact i have to put commnts for cod to be readable means it sucks
class WorldManagerNode(Node):
    def __init__(self):
        super().__init__("world_manager_node")
        self.get_logger().info(f"world_manager_node Started")
        
        self.declare_parameter("spawn_period", 5.0)
        self.spawn_period: float = self.get_parameter("spawn_period").value # type: ignore
        self.alive_turtles: list[TurtleInfo] = []
        
        # spawning random turtles
        self.spawn_turtle_serivce = self.create_service(Spawn, "spawn_turtle", self.cb_spawn_turtle) # the naming is fucken bad and confusing i really suck t this
        self.spawn_turtle_client = self.create_client(Spawn, "spawn")
        self.create_timer(self.spawn_period, self.spawn_random_turtle)
        
        # publishing alive turtles
        self.alive_turtles_publisher = self.create_publisher(AliveTurtles, "/alive_turtles", 10)
        self.create_timer(1, self.cb_publish_alive_turtles)
        
        # Killing Turtle
        self.kill_turtle_client = self.create_client(Kill, "kill")
        self.catch_turtle_service = self.create_service(Kill, "catch_turtle", self.cb_kill_turtle)
        
        self.cb_kill_turtle(Kill.Request(name="turtle1"), Kill.Response())
        
        
    # === publish alive status ====
    def cb_publish_alive_turtles(self):
        msg = AliveTurtles()
        msg.alive_turtles = self.alive_turtles
        
        self.alive_turtles_publisher.publish(msg)
    
    # ===spawn turtle===
    def cb_spawn_turtle(self, req: Spawn.Request, res: Spawn.Response):
        while not self.spawn_turtle_client.wait_for_service(1):
            self.get_logger().info("Turtle spawn service not ready yet...")
        
        future = self.spawn_turtle_client.call_async(req)
        future.add_done_callback(
            lambda fut: self.cb_turtle_spawned(fut, req.x, req.y, req.theta)
        )
        
        res.name = req.name
        return res
        # not sure how to return teh response here exactly... like i need to wait till the future is done right?
        
    def cb_turtle_spawned(self, future, x: float, y: float, theta: float):
        res: Spawn.Response = future.result()
        
        if res.name == "":
            self.is_killer_ready = False
            self.get_logger().error("Spawning failed, probably hunter name already taken")
            return
            
        turtle_info = TurtleInfo()
        turtle_info.name = res.name
        turtle_info.x = x
        turtle_info.y = y
        turtle_info.theta = theta
        
        
        self.alive_turtles.append(turtle_info)
        
    def spawn_random_turtle(self):
        req = Spawn.Request()
        req.name = ""
        req.x = float(random.randint(1, 10))
        req.y = float(random.randint(1, 10))
        req.theta = random.uniform(0, math.pi * 2)
        
        self.cb_spawn_turtle(req, Spawn.Response()) # calling it directly instead of using the service?
        
    # === killing turtle ===
    def cb_kill_turtle(self, req: Kill.Request, res: Kill.Response):
        while not self.kill_turtle_client.wait_for_service(1):
            self.get_logger().info("Turtle kill service not ready yet...")
        
        turtlesim_req = Kill.Request()
        turtlesim_req.name = req.name
        future = self.kill_turtle_client.call_async(turtlesim_req)
        future.add_done_callback(lambda fut: self.cb_kill_turtle_done(fut, req.name))
        
        return res
        
    def cb_kill_turtle_done(self, fut, name: str):
        for i, turtle_info in enumerate(self.alive_turtles):
            if turtle_info.name == name:
                self.get_logger().info(f"{name} was killed, nom nom nom...")
                self.alive_turtles.pop(i)
                return True
            
        self.get_logger().warning(f"{name} wasn't in teh alive list")
        return False
        
            
def main(args=None):
    rclpy.init(args=args)
    node = WorldManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    