import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

class ParamLoader(Node):
    def __init__(self):
        super().__init__('param_loader_client')
        self.client = self.create_client(SetParameters, '/steering_node/set_parameters')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ESP32 Parameter Server...')

    def send_params(self):
        req = SetParameters.Request()
        
        # Define your tuned values here
        kp_val = Parameter(name='kp', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=62.5))
        max_speed_val = Parameter(name='max_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=3800.0))
        
        req.parameters = [kp_val, max_speed_val]
        self.client.call_async(req)
        self.get_logger().info('Parameters sent to ESP32!')

def main():
    rclpy.init()
    loader = ParamLoader()
    loader.send_params()
    rclpy.shutdown()

if __name__ == '__main__':
    main()