import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.srv import LoadMap
import os

class MapLoaderNode(Node):
    def __init__(self):
        super().__init__('map_loader_node')

        # Declare and get the 'map_file' parameter
        self.declare_parameter('map_file', 'bme_ros2_navigation/maps/my_map.yaml')
        map_file_param = self.get_parameter('map_file').get_parameter_value().string_value

        # Extract package name and map file path from the parameter
        package_name, map_file_path = map_file_param.split('/', 1)

        # Get the absolute path to the map file
        try:
            package_path = get_package_share_directory(package_name)
            map_file = os.path.join(package_path, map_file_path)
            self.get_logger().info(f'Using map file: {map_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to find package: {package_name}. Error: {e}')
            return

        # Create a client for the /map_server/load_map service
        self.client = self.create_client(LoadMap, 'map_server/load_map')

        # Wait for the service to become available
        self.get_logger().info('Waiting for /map_server/load_map service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Call the service with the map file
        self.request = LoadMap.Request()
        self.request.map_url = map_file

    def send_request(self):
        self.get_logger().info(f'Calling /map_server/load_map with {self.request.map_url}...')
        return self.client.call_async(self.request)

'''
# Result code defintions
uint8 RESULT_SUCCESS=0
uint8 RESULT_MAP_DOES_NOT_EXIST=1
uint8 RESULT_INVALID_MAP_DATA=2
uint8 RESULT_INVALID_MAP_METADATA=3
uint8 RESULT_UNDEFINED_FAILURE=255
'''

def main(args=None):
    rclpy.init(args=args)
    node = MapLoaderNode()
    future = node.send_request()
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        if response.result == 0:
            node.get_logger().info('Map loaded successfully.')
        else:
            node.get_logger().error(f'Failed to load map: {response.result}')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()