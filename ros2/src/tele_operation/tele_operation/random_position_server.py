#!/usr/bin/env python

from tele_interface.srv import RandomPosition
import rclpy
import random

def handle_generate_random_position(request, response):
    response.x_random = random.uniform(request.x_min, request.x_max)
    response.y_random = random.uniform(request.y_min, request.y_max)
    print('randomx:', response.x_random)
    print('randomy:', response.y_random)
    return response

def random_position_server(args=None):

    rclpy.init(args=args)

    random_pos_node = rclpy.create_node('random_position_server')

    srv = random_pos_node.create_service(RandomPosition, 'generate_random_position', handle_generate_random_position)
    print('Ready to generate random position.')
    while rclpy.ok():
        rclpy.spin_once(random_pos_node)

    # Destroy the service attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    random_pos_node.destroy_service(srv)
    rclpy.shutdown()

if __name__ == '__main__':
    random_position_server()