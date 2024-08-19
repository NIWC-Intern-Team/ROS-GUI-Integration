#!/usr/bin/env python

from Phidget22.Phidget import *
from Phidget22.Devices.TemperatureSensor import *
from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.Devices.VoltageInput import *
import time

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64

class phidget_sensors(Node):
	
	def __init__(self):
		
		super().__init__('gus1_phidget_publisher')

		self._publish_temp0 = self.create_publisher(Float64, '/gus1/phidget/temperature0', 10)

		self._publish_amp0_data = self.create_publisher(Float64, '/gus1/phidget/amperage0', 10)
		self._publish_amp1_data = self.create_publisher(Float64, '/gus1/phidget/amperage1', 10)

		self._publish_volt0_data = self.create_publisher(Float64, '/gus1/phidget/voltage0', 10)
		self._publish_volt1_data = self.create_publisher(Float64, '/gus1/phidget/voltage1', 10)
		self._publish_volt2_data = self.create_publisher(Float64, '/gus1/phidget/voltage2', 10)

		def onTemperature0Change(self, temperature):
			temp0msg = Float64()
			temp0msg.data = temperature
			publish_temp0(temp0msg)
		
		def publish_temp0(temp0msg):
			self._publish_temp0.publish(temp0msg)

		def onAmperage0Change(self, amperage_data, sensor_value_info):
			ampD = Float64()
			ampD.data = amperage_data
			publish_amp0_data(ampD)

		def publish_amp0_data(ampD):
			self._publish_amp0_data.publish(ampD)

		def onAmperage1Change(self, amperage_data, sensor_value_info):
			ampD = Float64()
			ampD.data = amperage_data
			publish_amp1_data(ampD)

		def publish_amp1_data(ampD):
			self._publish_amp1_data.publish(ampD)
	
		def onVoltage0Change(self, voltage_data, sensor_value_info):
			voltD = Float64()
			voltD.data = voltage_data
			publish_volt0_data(voltD)

		def publish_volt0_data(voltD):
			self._publish_volt0_data.publish(voltD)
		
		def onVoltage1Change(self, voltage_data, sensor_value_info):
			voltD = Float64()
			voltD.data = voltage_data
			publish_volt1_data(voltD)

		def publish_volt1_data(voltD):
			self._publish_volt1_data.publish(voltD)

		def onVoltage2Change(self, voltage_data, sensor_value_info):
			voltD = Float64()
			voltD.data = voltage_data
			publish_volt2_data(voltD)

		def publish_volt2_data(voltD):
			self._publish_volt2_data.publish(voltD)

		#temp sensor
		temperatureSensor0 = TemperatureSensor()
		#amperage sensors (in order: Terminal 1, Terminal 2)
		voltageRatioInput0 = VoltageRatioInput()
		voltageRatioInput1 = VoltageRatioInput()
		#voltage sensors (in order: 5V sensor, Battery sensor, and e-stop sensor)
		voltageInput0 = VoltageInput()
		voltageInput1 = VoltageInput()
		voltageInput2 = VoltageInput()
		
		#temperatureSensor0.setIsHubPortDevice(True)
		voltageRatioInput0.setIsHubPortDevice(True)
		voltageRatioInput1.setIsHubPortDevice(True)
		voltageInput0.setIsHubPortDevice(True) 
		voltageInput1.setIsHubPortDevice(True) 
		voltageInput2.setIsHubPortDevice(True) 

		#as found in electronics box, subject to change
		temperatureSensor0.setHubPort(3)
		voltageRatioInput0.setHubPort(5)
		voltageRatioInput1.setHubPort(4)
		voltageInput0.setHubPort(0)
		voltageInput1.setHubPort(1)
		voltageInput2.setHubPort(2)
		
		temperatureSensor0.openWaitForAttachment(5000)
		voltageRatioInput0.openWaitForAttachment(5000)
		voltageRatioInput1.openWaitForAttachment(5000)
		voltageInput0.openWaitForAttachment(5000)
		voltageInput1.openWaitForAttachment(5000)
		voltageInput2.openWaitForAttachment(5000)

		voltageRatioInput0.setSensorType(VoltageRatioSensorType.SENSOR_TYPE_1122_AC) # other valid sensor types for this sensor include: SENSOR_TYPE_1122_DC, SENSOR_TYPE_1122_AC
		voltageRatioInput1.setSensorType(VoltageRatioSensorType.SENSOR_TYPE_1122_AC)
		voltageInput0.setSensorType(VoltageSensorType.SENSOR_TYPE_1135)
		voltageInput1.setSensorType(VoltageSensorType.SENSOR_TYPE_1135)
		voltageInput2.setSensorType(VoltageSensorType.SENSOR_TYPE_1135)

		#if change detected, publish change
		temperatureSensor0.setOnTemperatureChangeHandler(onTemperature0Change)
		voltageRatioInput0.setOnSensorChangeHandler(onAmperage0Change)
		voltageRatioInput1.setOnSensorChangeHandler(onAmperage1Change)
		voltageInput0.setOnSensorChangeHandler(onVoltage0Change)
		voltageInput1.setOnSensorChangeHandler(onVoltage1Change)
		voltageInput2.setOnSensorChangeHandler(onVoltage2Change)
		
		print("Phidgets are Publishing\nDo ctrl c to cancel")

		#what does this line do?
		#temperatureSensor0.close()

		

def main(args=None):
    rclpy.init(args=args)
    phidget_pub = phidget_sensors()
    rclpy.spin(phidget_pub)
    phidget_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
