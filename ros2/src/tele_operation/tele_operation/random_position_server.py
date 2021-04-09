#!/usr/bin/env python

from tele_interface.srv import RandomPosition
import rclpy
from rclpy.node import Node
import random

class RandomPosService(Node):

    def __init__(self):
        super().__init__('random_position_server')
        self.srv = self.create_service(RandomPosition, 'generate_random_position', self.random_position_callback)
        print('Ready to generate random position.')


    def random_position_callback(self, request, response):
        response.x_random = random.uniform(request.x_min, request.x_max)
        response.y_random = random.uniform(request.y_min, request.y_max)
        print('randomx:', response.x_random)
        print('randomy:', response.y_random)
        return response

def main(args=None):

    rclpy.init(args=args)

    random_pos_service = RandomPosService()

    rclpy.spin(random_pos_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()