#!/usr/bin/env python

import rospy
import std_msgs.msg
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField
import geometry_msgs

from collections import namedtuple
from math import *

import numpy

import serial
import serial.tools.list_ports
import time
import array
import math
import json
import urllib2
   
   
class Quaternion:
   def __init__(self,pitch,yaw,roll):
	   t0 = cos(yaw * 0.5);
	   t1 = sin(yaw * 0.5);
	   t2 = cos(roll * 0.5);
	   t3 = sin(roll * 0.5);
	   t4 = cos(pitch * 0.5);
	   t5 = sin(pitch * 0.5);   
      
	   self.w = t0 * t2 * t4 + t1 * t3 * t5;
	   self.x = t0 * t3 * t4 - t1 * t2 * t5;
	   self.y = t0 * t2 * t5 + t1 * t3 * t4;
	   self.z = t1 * t2 * t4 - t0 * t3 * t5;
	   
  

   
def data_fetch(full_api_url):
   url = urllib2.urlopen(full_api_url)
   output = url.read().decode('utf-8')
   print('output ' + output)
   raw_api_dict = json.loads(output)
   url.close()
   return raw_api_dict 
   
    
def userApi():
   return 'ca0e734f5c5107630ef9bf65b657e7f7'

def url_byName(city_id):
   unit = 'metric'
   api = 'http://api.openweathermap.org/data/2.5/weather?id='
   full_api_url = api + str(city_id) + '&mode=json&units' + unit + '&AAPID=' + 'ca0e734f5c5107630ef9bf65b657e7f7'
   return full_api_url
   'http://samples.openweathermap.org/data/2.5/weather?lat=35&lon=139&appid=b1b15e88fa797225412429c1c50c122a1'

def url_byCoord(lat,lng):
   api = 'http://api.openweathermap.org/data/2.5/weather?lat={0}&lon={1}&appid={2}'.format(lat,lng,'ca0e734f5c5107630ef9bf65b657e7f7')
   return api
   
def getAltitude(seaLevelPressure, atmosphericPressure, Temperature):
   return (((seaLevelPressure / atmosphericPressure) ** ( 1.0 / 5.257 ) - 1.0) * (Temperature + 273.15)) / 0.0065
   
# node send
def send():
   
   # init node     
   rospy.init_node('SendIMU',log_level=rospy.DEBUG)
   rate = rospy.Rate(10)
   
   navTransformPub = rospy.Publisher('/gps/fix',NavSatFix,queue_size=100)
   kalmanPub = rospy.Publisher('/imu/data_raw',Imu,queue_size=100)
   magnetField = rospy.Publisher('/imu/mag',MagneticField,queue_size=100)

   lat = 48.893583
   lng = 2.194122
   
   # get seal level pressure
   #seaLevelPressureString = data_fetch(url_byCoord(lat,lng)).get('main').get('pressure') 
   seaLevelPressure = 0.0 # float(seaLevelPressureString) 
         
   gyroscope_bias_x = 0.000050001
   gyroscope_bias_y = 0.001467536      
   gyroscope_bias_z = 0.002046174
         
   magnetometer_bias_x = 5.67524642536e-06
   magnetometer_bias_y = 1.7210987153e-05      
   magnetometer_bias_z = -6.53303760986e-05
   
   i = 0      
         
   while not rospy.is_shutdown():
      # get data from serial      
      ser = serial.Serial('/dev/ttyACM3',115200)
      line = ser.readline().strip()[1:]
      #line = "AZERTYUIOPAZERTYUIOAZERTYUIOAZERTYUIOPAZERTY"
      #line = [0] * 48
      
      if len(line) == 48:
         # Get data sensor
         floatsArray = array.array('f',line)
         
         # Extract pressure
         localPresureInt32 = array.array('I',line[-4:])[0]
         localPressureFloatPa = float(localPresureInt32)
         localPressureFloatHPa = localPressureFloatPa / 100.0
         localPressureFloatBar = localPressureFloatHPa / 10.0 
         
         #floatsArray[11] = getAltitude( localPressureFloatHPa,float(seaLevelPressure),22.0)
         #rospy.loginfo('altitude : {0}'.format(floatsArray[11]))
         #rospy.loginfo('Pa : {0} HPa: {1}'.format(localPressureFloatPa,localPressureFloatHPa))
         
         ##################################################
         ############### Create msg gps/fix ###############
         ##################################################
         
         # Header
         gpsMsg = NavSatFix()
         gpsMsg.header.frame_id  = "odom"
         gpsMsg.header.stamp     = rospy.Time.now()
         
         # Data
         gpsMsg.latitude   = floatsArray[9]
         gpsMsg.longitude  = floatsArray[10]
         gpsMsg.altitude   = floatsArray[11]

         # Covariance
         gpsMsg.position_covariance[0] = 100.0
         gpsMsg.position_covariance[4] = 100.0
         gpsMsg.position_covariance[8] = 100.0
         
         # Config
         gpsMsg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
         
         # Publication
         navTransformPub.publish(gpsMsg)
                  
         ###################################################
         ################## Create msg IMU #################
         ###################################################
         msg = Imu()
         msg.header.frame_id = "rocket"
         msg.header.stamp = rospy.Time.now()
         
         # NO ORIENTATION : HANDLE BY MAGDWIGHT

         # Angular_velocity
         # from degree /s to radian /s
         msg.angular_velocity.x = floatsArray[3] * 0.0174533 - gyroscope_bias_x         
         msg.angular_velocity.y = floatsArray[4] * 0.0174533 - gyroscope_bias_y
         msg.angular_velocity.z = floatsArray[5] * 0.0174533 - gyroscope_bias_z
         
         # Angular_velocity_covariance
         # 4000 degree/sec + 16 bits => 0.06103515625 degree => 0.001065264436029053 rad
         msg.angular_velocity_covariance[0] = 1.135e-6
         msg.angular_velocity_covariance[4] = 1.135e-6
         msg.angular_velocity_covariance[8] = 1.135e-6 
         
         # Linear_acceleration
         msg.linear_acceleration.x = floatsArray[0] * 9.8
         msg.linear_acceleration.y = floatsArray[1] * 9.8
         msg.linear_acceleration.z = floatsArray[2] * 9.8
         
         # Linear_acceleration_covariance
         # 32g + 16 bits => 4.8828125e-4 g => 4.78515625e-3 m/s^2
         msg.linear_acceleration_covariance[0] = 2.289772033e-5
         msg.linear_acceleration_covariance[4] = 2.289772033e-5
         msg.linear_acceleration_covariance[8] = 2.289772033e-5
         
         #kalmanPub.publish(msg)
         ####################################################
         ################## Magnet message ##################
         ####################################################
         
         magnetMessage = MagneticField()
         magnetMessage.header.frame_id = "rocket"
         magnetMessage.header.stamp = rospy.Time.now()
        
         magnetMessage.magnetic_field.x = floatsArray[7] * 1e-7 - magnetometer_bias_y
         magnetMessage.magnetic_field.y = floatsArray[6] * 1e-7 - magnetometer_bias_x
         magnetMessage.magnetic_field.z = -(floatsArray[8] * 1e-7 - magnetometer_bias_z)
         
         magnetMessage.magnetic_field_covariance[0] = 4.66034e-10
         magnetMessage.magnetic_field_covariance[4] = 4.66034e-10
         magnetMessage.magnetic_field_covariance[8] = 4.66034e-10
         
         #magnetField.publish(magnetMessage)
         
         ####################################################
         #################  END MAGNET  #####################
         ####################################################
         rate.sleep();

   
   
if __name__ == '__main__':
   try:
      rospy.logdebug('start')
      send()
   except rospy.ROSInterruptException:
      pass


  # string arduinoPort "/dev/ttyACM3/
   

