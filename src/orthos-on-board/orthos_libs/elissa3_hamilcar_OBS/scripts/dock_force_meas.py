#!/usr/bin/env python3
'''
Docking force measurement script
---------------------------------------------
ROS node for a HX711 load cell amplifier + load cell
    => Measures the force acting on a SINGLE docking pad
    => Create one node for EACH pad being measured
        Pads are differentiated by ID-Numbers 0, 1, 2, 3
'''
# ROS libraries
import rospy                        # ROS Python
from elissa3_hamilcar_obs.msg import ForceData
from std_msgs.msg import Float32    # Float message for force measurement
from std_srvs.srv import Trigger    # Trigger service for zeroing the sensor
from elissa3_hamilcar_obs.srv import ForceSetScale, ForceSetScaleResponse
# Other libraries
import RPi.GPIO as GPIO  # GPIO
from hx711 import HX711  # HX711 class


class dock_force_meas:
    def __init__(self, pad_nr=0, avg_window=5, rate=100, dout_pin=21, sck_pin=23, scale_ratio=1):
        # Setup the Raspberry Pi and the HX711 load cell amplifier
        rospy.loginfo(
            f'Force meas. pad {pad_nr}: dat_pin={dout_pin}, clk_pin={sck_pin}, rate={rate}, avg_window={avg_window}, scale={scale_ratio} ')
        # Set GPIO pin mode to BCM numbering
        GPIO.setmode(GPIO.BCM)
        # Data and clock pins
        self.dout_pin = dout_pin
        self.sck_pin = sck_pin
        # Last weight, raw and force objects to avoid measurement peaks
        self.last_weight = 0
        self.last_force = 0
        self.last_raw = 0
        # Instantiate HX711 object
        self.hx = HX711(self.dout_pin, self.sck_pin)
        # Conversion calibration factor from bit to grams, determined experimentally
        #       => DIFFERENT FOR EACH HX711 LOAD CELL AMP
        #       This value is from RFTP WS2021, p. 65
        self.calibrated = False
        self.ratio = scale_ratio
        self.hx.set_scale_ratio(self.ratio)
        # Zero the HX711 (no load should be applied)
        self.hx.zero()
        rospy.loginfo(f'Force meas. pad {pad_nr} calibrated')
        self.calibrated = True
        self.pad_nr = pad_nr
        # Averaging window for measurement readings
        self.avg_window = avg_window
        # Update rate and frequency for publishing messages
        self.rate = rate
        self.update_freq = 1.0/rate
        # Publisher, timer and service objects
        self.force_pub = rospy.Publisher('dock_force', ForceData, queue_size=1)  # old: 'dock_force_'+str(self.pad_nr
        self.pub_timer = rospy.Timer(rospy.Duration(self.update_freq), self.force_pub_cbk)
        self.zero_srv = rospy.Service('zero_dforce', Trigger, self.zero_srv_cbk)  # old: 'zero_dforce_'+str(self.pad_nr)
        self.zero_srv = rospy.Service('set_scale', ForceSetScale, self.set_scale_srv_cbk)

    # Publisher callback function to publish force measurements
    def force_pub_cbk(self, _):
        # Get the information from the load amplifier
        weight_mean = self.hx.get_weight_mean(self.avg_window)
        last_raw_data = self.hx.get_last_raw_data()
        # Calculate the mean force
        # F [N] = (meas[g] / 1000.0)[kg] * 9.81 [m/s^2]
        force_mean = (weight_mean / 1000.0) * 9.81
        # Zero everything above the limit
        if (abs(force_mean) > 10):
           force_mean = self.last_force
           weight_mean = self.last_weight
           last_raw_data = self.last_raw  # TODO Check if necessary
        # Create the message
        force_msg = ForceData()
        # Set the topic elements
        force_msg.readings = int(self.avg_window)
        force_msg.scale_ratio = float(self.ratio)
        force_msg.raw_data_mean = 0  # float(self.hx.get_raw_data_mean(self.avg_window))
        force_msg.data_mean = 0  # float(self.hx.get_data_mean(self.avg_window))
        force_msg.weight_mean = weight_mean
        force_msg.force_mean = float(force_mean)
        force_msg.last_raw_data = last_raw_data
        # Publish the message if calibrated
        if (self.calibrated == True):
            self.force_pub.publish(force_msg)
        # Set the current values as the last ones
        self.last_force = force_mean
        self.last_weight = weight_mean
        self.last_raw = last_raw_data

    # Service callback function to zero the force measurement device
    def zero_srv_cbk(self, req):
        self.calibrated = False
        self.hx.zero()
        self.calibrated = True
        return {'success': True, 'message': 'Force meas. pad '+str(self.pad_nr) + ' reset'}

    # Service callback to change the scale ratio of the sensor
    def set_scale_srv_cbk(self, msg):
        self.hx.set_scale_ratio(msg.scale_ratio)
        self.ratio = msg.scale_ratio
        return {'success': True, 'message': f'Force meas. pad {self.pad_nr} set to new scale {ratio}'}


if __name__ == '__main__':
    # get ROS parameters

    rospy.init_node('dock_force')
    pad = int(rospy.get_param('pad_nr', '0'))
    rate_val = int(rospy.get_param('rate', '100'))
    avg_win = int(rospy.get_param('avg_window', '1'))
    dat_pin = int(rospy.get_param('dat_pin', '21'))
    clk_pin = int(rospy.get_param('clk_pin', '23'))
    ratio = float(rospy.get_param('scale_ratio', '-217.5'))
    # rospy.init_node('dock_force_'+str(pad))

    # Instanciate freeflyer controller class
    force_meas_obj = dock_force_meas(
        pad_nr=pad, avg_window=avg_win, rate=rate_val, dout_pin=dat_pin, sck_pin=clk_pin, scale_ratio=ratio)

    rospy.spin()

    # Cleanup GPIO pins after ROS shutdown
    GPIO.cleanup()
