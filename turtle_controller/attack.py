#!/user/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
import random
from math import sqrt, pi, atan2

class Turtle(Node):

    #Initialize position
    posX = 0.0
    posY = 0.0
    theta = 0.0

    def __init__(self, name):
        super().__init__(name)
        
        #Create speed publisher, timer for applying function and position subscriber
        self.velocityPub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.attack)
        self.posSub = self.create_subscription(Pose, '/turtle1/pose', self.posCall, 10)

        #Spawn food turtles using /spawn
        self.clientS = self.create_client(Spawn, '/spawn')
        while not self.clientS.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn Service Timeout')
        self.reqSpawn = Spawn.Request()

        #Kill food turtles using /kill
        self.clientK = self.create_client(Kill, '/kill')
        while not self.clientK.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Kill Service Timeout')
        self.reqKill = Kill.Request()

        #Set first food turtle's index as 2 and find it
        self.count = 2
        self.findFood()

    def spawn(self):
        #Choose a random place within the window
        self.reqSpawn.x = random.uniform(0.0, 10.0)
        self.reqSpawn.y = random.uniform(0.0, 10.0)
        self.reqSpawn.theta = 0.0

        #Spawn and set kill request
        self.reqKill.name = f'turtle{self.count}'
        self.future = self.clientS.call_async(self.reqSpawn)
        self.get_logger().info(f'Spawn at ({self.reqSpawn.x}, {self.reqSpawn.y})')

    def findFood(self):
        #spawn a food turtle first
        self.spawn()

        #get the location of food turtle
        self.foodX = self.reqSpawn.x
        self.foodY = self.reqSpawn.y
        self.get_logger().info(f'Food at ({self.foodX}, {self.foodY})')

    def kill(self, count):
        #set kill request again
        self.reqKill.name = f'turtle{count}'
        self.future = self.clientK.call_async(self.reqKill)

    def posCall(self, pose_msg = Pose()):
        #Deliver the message that contains current position
        self.posX = pose_msg.x
        self.posY = pose_msg.y
        self.theta = pose_msg.theta

    def attack(self):
        #calculate distance between two turtles
        self.distX = self.foodX - self.posX
        self.distY = self.foodY - self.posY
        distance = sqrt(self.distX**2 + self.distY**2)

        #set speed according to current angle
        speed = Twist()
        if distance > 0.1:
            speed.linear.x = 2.0
            angle = atan2(self.distY, self.distX)
            angleDif = angle - self.theta
            if angleDif > pi:     
                angleDif -= 2*pi
            elif angleDif < -pi:  
                angleDif += 2*pi
            speed.angular.z = 6*(angleDif)

        #stop moving and start killing when approaching food
        else:
            speed.angular.z = 0.0
            speed.linear.x = 0.0
            self.kill(self.count)
            self.get_logger().info("Food eaten")
            self.count += 1
            self.findFood()

        self.velocityPub.publish(speed)

def main(args=None):
    rclpy.init(args=args)
    node = Turtle('attack')
    while rclpy.ok():
        rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()