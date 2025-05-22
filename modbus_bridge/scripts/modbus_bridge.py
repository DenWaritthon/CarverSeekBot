#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from pyModbusTCP.client import ModbusClient
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class ModbusBridge(Node):
    def __init__(self):
        super().__init__('ModbusBridge')

        self.declare_parameter('ip', '192.168.1.190')
        self.declare_parameter('port', 502)

        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        self.client = ModbusClient(host=ip, port=port, auto_open=True)
        self.get_logger().info(f'Connecting to Controllino @ {ip}:{port}')

        # Publishers
        self.emer_pub = self.create_publisher(Bool, '/sensor/emer', 10)
        self.b_l_pub = self.create_publisher(Bool, '/sensor/limit_b_l', 10)
        self.b_r_pub = self.create_publisher(Bool, '/sensor/limit_b_r', 10)
        self.f_l_pub = self.create_publisher(Bool, '/sensor/limit_f_l', 10)
        self.f_r_pub = self.create_publisher(Bool, '/sensor/limit_f_r', 10)

        # Service Server
        # self.create_subscription(String, '/command/led', self.led_cb, 10)
        self.create_service(SetBool, '/command/motor', self.motor_cb)
        
        # Polling timer
        self.create_timer(0.05, self.poll_sensors)

    def poll_sensors(self):
        if not self.client.is_open:  # Fixed: removed parentheses
            self.client.open()
        coils = self.client.read_coils(0, 5)
        if coils:
            self.emer_pub.publish(Bool(data=coils[0]))
            self.b_l_pub.publish(Bool(data=coils[1]))
            self.b_r_pub.publish(Bool(data=coils[2]))
            self.f_l_pub.publish(Bool(data=coils[3]))
            self.f_r_pub.publish(Bool(data=coils[4]))

    # def led_cb(self, msg: String):
    #     color_map = {
    #         'red': 10,
    #         'green': 11,
    #         'blue': 12,
    #         'off': 13,
    #         'white': 14,
    #         'purple': 15,
    #         'yellow': 16,
    #         'indigo': 17,
    #     }
    #     idx = color_map.get(msg.data.lower())
    #     if idx is not None:
    #         self.client.write_single_coil(idx, True)

    def motor_cb(self, req, res):
        if req.data:
            self.client.write_single_coil(20, True)
            res.success = True
            res.message = "Motor started"
        else:
            self.client.write_single_coil(21, True)
            res.success = True
            res.message = "Motor stopped"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ModbusBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
