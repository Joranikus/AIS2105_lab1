import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64

class PIDController:
    def __init__(self, p, i, d, reference):
        self.p = p
        self.i = i
        self.d = d
        self.reference = reference
        self.voltage = 0.0
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, measured_value, dt):
        error = self.reference - measured_value
        self.integral += error * dt

        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0

        self.voltage = self.p * error + self.i * self.integral + self.d * derivative
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
                ('reference', 0.0)
            ])

        self.pid = PIDController(
            self.get_parameter('p').value,
            self.get_parameter('i').value,
            self.get_parameter('d').value,
            self.get_parameter('reference').value
        )

        self.publisher_ = self.create_publisher(Float64, 'voltage', 10)
        self.subscription = self.create_subscription(Float64, 'measured_angle', self.measurement_listener, 10)
        self.last_time = None
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create a timer that calls status() every 0.5 seconds
        self.status_timer = self.create_timer(0.5, self.status)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'p' and param.value >= 0.0:
                self.pid.p = param.value
            elif param.name == 'i' and param.value >= 0.0:
                self.pid.i = param.value
            elif param.name == 'd' and param.value >= 0.0:
                self.pid.d = param.value
            elif param.name == 'reference':
                self.pid.reference = param.value

        return rclpy.parameter.SetParametersResult(successful=True)

    def measurement_listener(self, msg):
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
        self.get_logger().info(
            f"[STATUS] p: {self.pid.p:.3f}, i: {self.pid.i:.3f}, d: {self.pid.d:.3f}, "
            f"reference: {self.pid.reference:.3f}, voltage: {self.pid.voltage:.3f}, "
            f"integral: {self.pid.integral:.3f}, prev_error: {self.pid.prev_error:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
