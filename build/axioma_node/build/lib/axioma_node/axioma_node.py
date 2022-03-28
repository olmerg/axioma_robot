# Copyright 2021 LMA, UNICAMP
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import time
import serial
import math

from sensor_msgs.msg import JointState
from nav2_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


from tf2_ros import TransformBroadcaster

import tf_transformations

from geometry_msgs.msg import Twist
from std_msgs.msg import Header


class AxiomaNode(Node):

    def __init__(self):
        super().__init__('axioma_node')
        self.joint_state_pub_ = self.create_publisher(
            JointState, 'joint_states', 10)
        self.odom_pub_ = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_vel_sub_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            1)
        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        try:
            self.ser = serial.Serial(timeout=0.045, write_timeout=0.001)
            self.ser.baudrate = 115200
            self.ser.port = '/dev/ttyACM0'
            self.ser.open()
            time.sleep(1)
            self.simulation = False
        except:
            self.simulation = True
            self.get_logger().warning(
                'Error al abrir el puerto Serial, usando simulación cinemática')

        # TODO: adicionar parametros (configurar correctamente)
        self.t_s = 0.05  # seconds
        self.L = 0.12  # distancia entre las ruedas traseras
        self.r = .04  # radio de las ruedas
        # ODOMETRIA posicion inicial del robot
        self.frame_robot = 'base_link'
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v_x = 0
        self.w_z = 0
        self.theta_l_ant = 0
        self.theta_r_ant = 0
        # odometria fin variables
        self.timer = self.create_timer(self.t_s, self.timer_callback)
        self.v_xd = 0
        self.w_zd = 0
        self.theta_l = 0
        self.theta_r = 0
        self.time_1 = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        """
        Esta funciòn recibe el tópico para comandar el robot
        """
        self.v_xd = msg.linear.x
        self.w_zd = msg.angular.z

    def clamp(self, num, min_value, max_value):
        """
        Función para limitar el valor de las señales
        """
        return max(min(num, max_value), min_value)

    def timer_callback(self):
        """
        Esta función es periodica y es la encargada de enviar el comando y recibir la información del serial
        """

        w_ld = 1/self.r * (self.v_xd - self.L*self.w_zd)
        w_rd = 1/self.r * (self.v_xd + self.L*self.w_zd)
        my_time = self.get_clock().now()

        delta_t = (my_time-self.time_1).nanoseconds/1000000000
        if not self.simulation:
            None
            w_l_ = int(self.clamp(w_ld*10, 0, 255))
            w_r_ = int(self.clamp(w_rd*10, 0, 255))
            command = 1
            comandos = 'z='+str(command)+' vl='+str(w_l_)+' vr='+str(w_r_)+'\n'
            self.ser.write(comandos.encode())
            # lectura de los datos
            line = self.ser.read_until().decode("utf-8")

            self.w_l = -1.0
            self.w_r = -1.0
            if len(line) > 0:
                datos = line[:-2].split(',')

                if len(datos) == 3:
                    self.w_l = float(datos[0])/10.
                    self.w_r = float(datos[1])/10.
                    print(self.w_l, self.w_r)
                    # TODO: use datos[2]=time para errores y usar try except
                else:
                    print("error de recepción de datos")

            else:
                print('error de recepción de datos')
            self.theta_l += self.w_l * delta_t
            self.theta_r += self.w_r * delta_t

        else:
            # modelo cinemático para simular el robot
            self.w_l = w_ld
            self.w_r = w_rd
            self.theta_l += self.w_l * delta_t
            self.theta_r += self.w_r * delta_t

        # TODO: adicionar el nombre de los joint the cada rueda del urdf
        joints = JointState()
        joints.header = Header()
        joints.header.frame_id = "base_link"
        joints.name = ['l_tyre', 'r_tyre']
        joints.position = [self.theta_l, self.theta_r]
        joints.velocity = [self.w_l, self.w_r]
        joints.effort = [-1.0, -1.0]
        self.joint_state_pub_.publish(joints)
        self.time_1 = my_time

# estimar la odometria a partir de los encoder
        dw_l = self.theta_l-self.theta_l_ant
        dw_r = self.theta_r-self.theta_r_ant
        self.theta_l_ant = self.theta_l
        self.theta_r_ant = self.theta_r
        delta_s = self.r * (dw_l + dw_r) / 2.0
        delta_theta = self.r * (dw_l - dw_r) / self.L
        self.x += delta_s * math.cos(self.theta + (delta_theta / 2.0))
        self.y += delta_s * math.sin(self.theta + (delta_theta / 2.0))
        self.theta += delta_theta
        self.publish_odom(my_time)

    def publish_odom(self, my_time):
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = self.frame_base_robot
        odom_msg.header.stamp = my_time

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0

        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = self.v_x
        odom_msg.twist.twist.angular.z = self.w_z
        self.odom_pub_.publish(odom_msg)
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = my_time
        t.header.frame_id = 'odom'
        t.child_frame_id = odom_msg.child_frame_id

        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w

        # Send the transformation
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    axioma_node = AxiomaNode()

    rclpy.spin(axioma_node)

    # Destroy the node explicitly (optional)
    axioma_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
