from autonomous_vehicle.srv import *
import rospy
import random

def handle_generate_random_position(RandomPositionRequest):
    res = RandomPositionResponse()
    res.x_random = random.uniform(RandomPositionRequest.x_min, RandomPositionRequest.x_max)
    res.y_random = random.uniform(RandomPositionRequest.y_min, RandomPositionRequest.y_max)
    return res

def random_position_server():
    rospy.init_node('random_position_server')
    srv = rospy.Service('generate_random_position', RandomPosition, handle_generate_random_position)
    print('Ready to generate random position.')
    rospy.spin()

if __name__ == '__main__':
    random_position_server()