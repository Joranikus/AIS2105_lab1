import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult

class PIDController:
    def __init__(self, p, i, d, reference, max_voltage=12.0, min_voltage=-12.0):
        self.p = p
        self.i = i
        self.d = d
        self.reference = reference
        self.max_voltage = max_voltage
        self.min_voltage = min_voltage
        
        self.voltage = 0.0
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, measured_value, dt):
        error = self.reference - measured_value
        
        potential_integral = self.integral + error * dt
        
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0

        potential_voltage = (self.p * error) + (self.i * potential_integral) + (self.d * derivative)

        if potential_voltage > self.max_voltage:
            self.voltage = self.max_voltage
        elif potential_voltage < self.min_voltage:
            self.voltage = self.min_voltage
        else:
            self.voltage = potential_voltage
            self.integral = potential_integral

        self.prev_error = error

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('p', 1.0),
                ('i', 0.0),
                ('d', 0.0),
                ('reference', 0.0),
                ('max_voltage', 12.0),
                ('min_voltage', -12.0)
            ])

        self.pid = PIDController(
            p=self.get_parameter('p').value,
            i=self.get_parameter('i').value,
            d=self.get_parameter('d').value,
            reference=self.get_parameter('reference').value,
            max_voltage=self.get_parameter('max_voltage').value,
            min_voltage=self.get_parameter('min_voltage').value
        )

        self.publisher_ = self.create_publisher(Float64, 'voltage', 10)
        self.subscription = self.create_subscription(
            Float64, 
            'measured_angle', 
            self.measurement_listener, 
            10
        )
        
        self.last_time = None
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.status_timer = self.create_timer(0.5, self.status)

    def parameter_callback(self, params):
        """Handle parameter updates"""
        for param in params:
            if param.name == 'p' and param.value >= 0.0:
                self.pid.p = param.value
            elif param.name == 'i' and param.value >= 0.0:
                self.pid.i = param.value
            elif param.name == 'd' and param.value >= 0.0:
                self.pid.d = param.value
            elif param.name == 'reference':
                self.pid.reference = param.value
            elif param.name == 'max_voltage':
                self.pid.max_voltage = param.value
            elif param.name == 'min_voltage':
                self.pid.min_voltage = param.value

        return SetParametersResult(successful=True)

    def measurement_listener(self, msg):
        """Process new measurements"""
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            dt = 0.0
            self.last_time = current_time
        else:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time
        
        self.pid.update(msg.data, dt)
        
        voltage_msg = Float64()
        voltage_msg.data = self.pid.voltage
        self.publisher_.publish(voltage_msg)

    def status(self):
        """Log current controller state"""
        self.get_logger().info(
            f"[STATUS] P: {self.pid.p:.3f}, I: {self.pid.i:.3f}, D: {self.pid.d:.3f}, "
            f"Reference: {self.pid.reference:.3f}, Voltage: {self.pid.voltage:.3f}, "
            f"Integral: {self.pid.integral:.3f}, "
            f"Voltage Limits: [{self.pid.min_voltage:.3f}, {self.pid.max_voltage:.3f}]"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()