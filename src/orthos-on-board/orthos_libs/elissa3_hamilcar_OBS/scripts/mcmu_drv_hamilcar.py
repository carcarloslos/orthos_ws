#! /usr/bin/python3
""" 
This module connects to the :samp:`MCMU` via a serial interface. It receives the target
thrust vector :samp:`actuator_force_goal` and computes and sends control signals for the hardware.

.. image:: images/mcmu_side.png
    :width: 400
    :alt: Alternative text


Todo:
    * Furthermore it recieves information about the actuators current states and publishes them to the ROS network

"""
import serial
import struct
import rospy
from std_msgs.msg import Float32MultiArray

class mcmu_driver:
    """
    Class that provides an abstract representation of the :samp:`MCMU`

    Args:
        serial_port (str): String representing the serial port the :samp:`MCMU` is attached to (e.g. :file:`/dev/ttyUSB0` )
        baudrate (int): The baudrate used for communication. It has to match the baudrate used in the :samp:`MCMUs` firmware
    """
    def __init__(self,serial_port,baudrate):
        self.conn=serial.Serial(serial_port,baudrate)

    def transmit_receive(self,us_goal):
        """
        In one step, send the updated vector of target rpms to the :samp:`MCMU` and receive the vector of actual rpms

        Args:
            us_goal (array-like):array of length 8 of target rpm values

        Returns:
            Vector of actual rpm values send from the :samp:`MCMU`

        """
        #transmit rpm goal and wait for the current rpm
        tx_buf=bytearray()
        for us_val in us_goal:
            tx_buf.extend(struct.pack("<f", us_val))

        self.conn.write(tx_buf)
        rospy.sleep(0.01)
        ans=self.conn.read(32)
        return struct.unpack("<ffffffff", ans)

class mcmu_driver_node:
    """
    ROS node that wraps around the mcmu_driver and enables communication to other nodes.

    Args:
        serial_port (str): String representing the serial port the :samp:`MCMU` is attached to (e.g. :file:`/dev/ttyUSB0` )
        baudrate (int): The baudrate used for communication. It has to match the baudrate used in the :samp:`MCMUs` firmware
        u_freq (int): update frequency.
    """
    def __init__(self,serial_port,baudrate, u_freq=100):
        self.actuator_force_goal=[0,0,0,0,0,0,0,0]
        self.actuator_force=[0,0,0,0,0,0,0,0]

        #mcmu interface
        self.mcmu_if=mcmu_driver(serial_port,baudrate)
        rospy.loginfo("Connected to: "+serial_port)
        rospy.sleep(1) #wait for hw

        #subscribe to the fams output
        self.force_sub=rospy.Subscriber("actuator_force_goal", Float32MultiArray, self.actuator_force_rec_clbk)

        self.update_if_timer=rospy.Timer(rospy.Duration(1.0/u_freq), self.tx_rx_clbk)

    def tx_rx_clbk(self,event):
        r"""
        This callback function sends and receives at the rate sepcified in the constructor. Prior to sending a new rpm goal vector it checks, that its values no not exceed the maximum rpms of the :samp:`MCMU` (which is set to 20000). It assumes a linear relation of thrust and rpm:
        for amilcar, a linear relation of target rpm to pwm signal is used (for now). A more accurate way to do this would be to consider the battery voltage, but thats TBD.

        """
        us_goal=[1000 + us_val*(1000) for us_val in self.actuator_force_goal]
        #rpm limiting


        us_out=[]
        for us_val in us_goal:
            if us_val > 1600:
                us_out.append(1600)
            else:
                us_out.append(us_val)

        #self.actuator_force=[us_val*(4.0/20000.0) for us_val in actuator_rpm]
        actuator_us=self.mcmu_if.transmit_receive(us_out)

    def actuator_force_rec_clbk(self,msg):
        """
        Callback function that reacts to updates to the :samp:`actuator_force_goal` topic.
        """
        fmap=list(msg.data)
        self.actuator_force_goal=[fmap[0],fmap[1],fmap[2],fmap[3],fmap[4],fmap[5],fmap[6],fmap[7]]

if __name__=="__main__":
    rospy.init_node("mcmu_drv")
    port = rospy.get_param('~port',"/dev/ttyACM0")
    mcmu_drv=mcmu_driver_node(port,9600)
    rospy.spin()
