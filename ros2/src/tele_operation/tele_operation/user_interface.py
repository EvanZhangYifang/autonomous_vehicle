#!/usr/bin/env python

import sys


import geometry_msgs.msg
import rclpy
from rclpy.node import Node
from tele_interface.srv import RandomPosition
from std_srvs.srv import SetBool
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

randomMove={
    'g':(0,0,0,0),
}

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

class user_interface_clients(Node):

    def __init__(self):
        super().__init__('teleop_twist_keyboard')

        self.publisher = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)

        # self.cli_pos = self.create_client(RandomPosition, 'generate_random_position')
        self.cli_mode = self.create_client(SetBool, '/control_mode_switch')

        # self.declare_parameter('des_pos_x', 0)
        # self.declare_parameter('des_pos_y', 0)

        # while not self.cli_pos.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        while not self.cli_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req_mode = SetBool.Request()
        # self.req_pos = RandomPosition.Request()

    def set_control_mode(self, control_mode):
        self.req_mode.data = control_mode
        self.mode_response = self.cli_mode.call_async(self.req_mode)

    # def get_random_pos(self):
    #     self.req_pos.x_min = -8.0
    #     self.req_pos.x_max = 8.0
    #     self.req_pos.y_min = -8.0
    #     self.req_pos.y_max = 8.0
    #     self.future = self.cli_pos.call_async(self.req_pos)

        # while rclpy.ok():
        #     if self.future.done():
        #         pos_response = self.future.result()
        #         print(pos_response)
        #         new_des_pos_x = rclpy.parameter.Parameter(
        #             'des_pos_x',
        #             rclpy.Parameter.Type.DOUBLE,
        #             pos_response.x_random
        #         )
        #         new_des_pos_y = rclpy.parameter.Parameter(
        #             'des_pos_y',
        #             rclpy.Parameter.Type.DOUBLE,
        #             pos_response.y_random
        #         )
        #         all_new_parameters = [new_des_pos_x, new_des_pos_y]
        #         self.set_parameters(all_new_parameters)
        #         print('Go to position x: ', pos_response.x_random,'y: ', pos_response.y_random)
        #         break
        #     print('Wait for service finish its job.')

    # def set_random_pos(self):
        # new_des_pos_x = rclpy.parameter.Parameter(
        #             'des_pos_x',
        #             rclpy.Parameter.Type.DOUBLE,
        #             pos_response.x_random
        #         )
        # new_des_pos_y = rclpy.parameter.Parameter(
        #     'des_pos_y',
        #     rclpy.Parameter.Type.DOUBLE,
        #     pos_response.y_random
        # )
        # all_new_parameters = [new_des_pos_x, new_des_pos_y]
        # self.set_parameters(all_new_parameters)
        # print('Go to position x: ', pos_response.x_random,'y: ', pos_response.y_random)

    def pub_mesg(self, mesg):
        self.publisher.publish(mesg)

        


def main():

    # global pos_response

    settings = saveTerminalSettings()

    rclpy.init()

    node = user_interface_clients()


    speed = 0.5
    turn = 2.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            print('00')
            key = getKey(settings)
            print('0')
            if key in moveBindings.keys():
                print('1')

                node.set_control_mode(False)

                print('1.1')

                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                print('2')

                node.set_control_mode(False)

                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15

            elif key in randomMove.keys():
                print('3')

                # node.set_control_mode(False)
                node.set_control_mode(True)

                print('Switch to random position mode.')

                # node.get_random_pos()

                # while rclpy.ok():
                #     rclpy.spin_once(node)
                #     if node.future.done():
                #         pos_response = node.future.result()
                #         print(pos_response)
                #         node.set_random_pos()
                #         break

                # set_random_pos()
            else:
                print('4')

                node.set_control_mode(False)

                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            node.pub_mesg(twist)

    except Exception as e:
        print(e)

    finally:
        print('5')
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        node.pub_mesg(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
