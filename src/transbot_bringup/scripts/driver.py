from Transbot_Lib import Transbot
from transbot_bringup.cfg import PIDparamConfig
from dynamic_reconfigure.server import Server
from transbot_msgs.msg import Edition, Battery
from transbot_msgs.srv import RobotArm, RGBLight, Buzzer, Headlight, PWMServo
from transbot_msgs.msg import Arm, Joint
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy
import sys
sys.path.append("/home/yahboom/py_install-V3.2.5/py_install")


class TransbotDriver(Node):
    def __init__(self):
        super().__init__('driver_node')
        self.create_subscription(
            Int32, '/control_mode', self.control_mode_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Arm, '/TargetAngle', self.sub_armcallback, 10)
        self.create_subscription(
            PWMServo, '/PWMServo', self.sub_PWMServocallback, 10)
        self.create_subscription(
            PIDparamConfig, 'parameter_updates', self.dynamic_reconfigure_callback, 10)
        self.create_service(RobotArm, '/CurrentAngle', self.RobotArmcallback)
        self.create_service(RGBLight, '/RGBLight', self.RGBLightcallback)
        self.create_service(Buzzer, '/Buzzer', self.Buzzercallback)
        self.create_service(Headlight, '/Headlight', self.Headlightcallback)
        self.create_publisher(Edition, '/edition', 10)
        self.create_publisher(Battery, '/voltage', 10)
        self.create_publisher(Twist, '/transbot/get_vel', 10)
        self.create_publisher(Imu, '/transbot/imu', 10)
        self.bot = Transbot(com="/dev/ttyAMA0")
        self.bot.create_receive_threading()
        self.bot.set_uart_servo_angle(9, 90)
        self.linear_max = 0.4
        self.linear_min = 0.0
        self.angular_max = 2.0
        self.angular_min = 0.0

    def control_mode_callback(self, msg):
        # Example: Control mode callback
        control_mode = msg.data
        if control_mode == 0:
            self.bot.set_mode("manual")
        elif control_mode == 1:
            self.bot.set_mode("auto")

    def cmd_vel_callback(self, msg):
        # Example: Twist message callback for controlling the robot's movement
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        self.bot.set_car_motion(linear_speed, angular_speed)

    def sub_armcallback(self, msg):
        # Example: Arm control callback
        for joint in msg.joint:
            joint_id = joint.id
            joint_angle = joint.angle
            run_time = joint.run_time
            self.bot.set_uart_servo_angle(joint_id, joint_angle, run_time)

    def sub_PWMServocallback(self, msg):
        # Example: PWM servo control callback
        servo_id = msg.id
        servo_angle = msg.angle
        if self.CameraDevice == "astra":
            if servo_id == 1:
                servo_angle = max(60, min(120, servo_angle))
        if servo_id == 2:
            servo_angle = min(140, servo_angle)
        self.bot.set_pwm_servo(servo_id, servo_angle)

    def RobotArmcallback(self, request, response):
        # Example: RobotArm service callback
        joints = self.bot.get_uart_servo_angle_array()
        for i in range(3):
            joint = Joint()
            joint.id = i + 7
            joint.angle = joints[i]
            joint.run_time = 500
            if joints[i] >= 0:
                response.RobotArm.joint.append(joint)
        return response

    def RGBLightcallback(self, request, response):
        # Example: RGBLight service callback
        effect = request.effect
        speed = request.speed
        self.bot.set_colorful_effect(effect, speed, parm=1)
        response.result = True
        return response

    def Buzzercallback(self, request, response):
        # Example: Buzzer service callback
        self.bot.set_beep(request.buzzer)
        response.result = True
        return response

    def Headlightcallback(self, request, response):
        # Example: Headlight service callback
        self.bot.set_floodlight(request.Headlight)
        response.result = True
        return response

    def dynamic_reconfigure_callback(self, config, level):
        # Example: Dynamic reconfigure callback
        self.linear_max = config.linear_max
        self.linear_min = config.linear_min
        self.angular_max = config.angular_max
        self.angular_min = config.angular_min
        return config

    def pub_data(self):
        # Example: Publish data such as battery voltage, gyroscope data, and speed
        rate = self.create_rate(20)
        while rclpy.ok():
            ax, ay, az = self.bot.get_accelerometer_data()
            gx, gy, gz = self.bot.get_gyroscope_data()
            velocity, angular = self.bot.get_motion_data()
            voltage = self.bot.get_battery_voltage()

            battery = Battery()
            battery.Voltage = voltage
            self.publish('/voltage', battery)

            imu = Imu()
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            self.publish('/transbot/imu', imu)

            twist = Twist()
            twist.linear.x = velocity
            twist.angular.z = angular
            self.publish('/transbot/get_vel', twist)

            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    driver = TransbotDriver()
    driver.pub_data()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
