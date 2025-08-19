#!/usr/bin/env python3
"""
Custom TurtleBot3 Teleop with uiojklm,. keys
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
🤖 TurtleBot3 Custom Teleop Control
================================

Movement Controls:
   u    i    o
   j    k    l
   m    ,    .

u/o : turn left/right while moving forward
i   : move forward  
j/l : turn left/right
k   : stop
m/. : turn left/right while moving backward
,   : move backward

CTRL+C to quit
"""

moveBindings = {
    'i': (1, 0),   # forward
    'o': (1, -1),  # forward + turn right  
    'j': (0, 1),   # turn left
    'l': (0, -1),  # turn right
    'u': (1, 1),   # forward + turn left
    ',': (-1, 0),  # backward
    '.': (-1, -1), # backward + turn right
    'm': (-1, 1),  # backward + turn left
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1),
    'x': (0.9, 1),
    'e': (1, 1.1),
    'c': (1, 0.9),
}

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0
        self.th = 0
        self.status = 0
        self.count = 0
        self.acc = 0.1
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0
        
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setcraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)

    def run(self):
        try:
            print(msg)
            print(self.vels(self.speed, self.turn))
            
            while True:
                key = self.getKey()
                
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.th = moveBindings[key][1]
                    self.count = 0
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    self.count = 0
                    
                    print(self.vels(self.speed, self.turn))
                    if (self.status == 14):
                        print(msg)
                    self.status = (self.status + 1) % 15
                elif key == ' ' or key == 'k':
                    self.x = 0
                    self.th = 0
                    self.control_speed = 0
                    self.control_turn = 0
                else:
                    self.count = self.count + 1
                    if self.count > 4:
                        self.x = 0
                        self.th = 0
                    if (key == '\x03'):  # Ctrl+C
                        break

                self.target_speed = self.speed * self.x
                self.target_turn = self.turn * self.th

                if self.target_speed > self.control_speed:
                    self.control_speed = min(self.target_speed, self.control_speed + 0.02)
                elif self.target_speed < self.control_speed:
                    self.control_speed = max(self.target_speed, self.control_speed - 0.02)
                else:
                    self.control_speed = self.target_speed

                if self.target_turn > self.control_turn:
                    self.control_turn = min(self.target_turn, self.control_turn + 0.1)
                elif self.target_turn < self.control_turn:
                    self.control_turn = max(self.target_turn, self.control_turn - 0.1)
                else:
                    self.control_turn = self.target_turn

                twist = Twist()
                twist.linear.x = self.control_speed
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.control_turn
                self.pub.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = TeleopTwistKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
