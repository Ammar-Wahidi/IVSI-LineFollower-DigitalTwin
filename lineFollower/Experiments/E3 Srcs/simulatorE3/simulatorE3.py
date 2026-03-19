#!/usr/bin/env python3
# Use Python 3 interpreter

from __future__ import print_function
# Ensures print() behaves consistently (mainly for Python 2 compatibility)

import struct   # For packing/unpacking binary data (used in CAN communication)
import sys
import argparse # For command-line arguments
import math

# Path to VSI Python gateway APIs
PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

# Import VSI APIs (simulation + CAN interface)
import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway


# ============================================================
# SIGNAL CLASS (INPUTS / OUTPUTS)
# ============================================================
class MySignals:
	def __init__(self):
		# Inputs (received from CAN)
		self.v = 0        # Linear velocity
		self.omega = 0    # Angular velocity

		# Outputs (sent to CAN)
		self.x = 0        # Position X
		self.y = 0        # Position Y
		self.theta = 0    # Orientation angle


# ============================================================
# USER GLOBAL VARIABLES (INITIAL CONDITIONS + PARAMETERS)
# ============================================================
import random
import math

# Initial robot state (randomized)
x = random.uniform(-0.5, 0.5)

# y initialized away from center (avoid near-zero region)
y = random.choice([
    random.uniform(0.25, 0.5),    # positive region
    random.uniform(-0.5, -0.25)   # negative region
])

theta = random.uniform(-0.3, 0.3)  # initial orientation

dt = 0.1              # time step
noise_level = 0.1     # noise amplitude
disturbance_mag = 2   # sudden disturbance magnitude


# ============================================================
# MAIN SIMULATOR CLASS
# ============================================================
class Simulator:

	def __init__(self, args):
		# Connection parameters
		self.componentId = 0
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50101
        
		# Simulation control
		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0
        
		# Communication buffers
		self.receivedNumberOfBytes = 0
		self.receivedPayload = []
		self.mySignals = MySignals()


	# ============================================================
	# MAIN SIMULATION LOOP
	# ============================================================
	def mainThread(self):

		# Connect to VSI simulation server
		dSession = vsiCommonPythonApi.connectToServer(
			self.localHost, self.domain, self.portNum, self.componentId
		)

		# Initialize CAN interface
		vsiCanPythonGateway.initialize(dSession, self.componentId)

		try:
			# Wait for simulation reset signal
			vsiCommonPythonApi.waitForReset()

			# Update timing parameters
			self.updateInternalVariables()

			if(vsiCommonPythonApi.isStopRequested()):
				raise Exception("stopRequested")

			nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()

			# Main time loop
			while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

				# ====================================================
				# ROBOT KINEMATICS MODEL
				# ====================================================
				global x, y, theta

				v     = self.mySignals.v
				omega = self.mySignals.omega

				# Differential drive kinematics
				x     += v * math.cos(theta) * dt
				y     += v * math.sin(theta) * dt
				theta += omega * dt

				# Add Gaussian noise
				x += noise_level * random.gauss(0, 1) * dt
				y += noise_level * random.gauss(0, 1) * dt

				# Random disturbance (rare sudden rotation)
				if random.random() < 0.005:
					theta += disturbance_mag * random.choice([-1, 1])

				# Update output signals
				self.mySignals.x     = x
				self.mySignals.y     = y
				self.mySignals.theta = theta

				# ====================================================
				# RECEIVE INPUTS FROM CAN
				# ====================================================
				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")

				# Receive velocity (v)
				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(
					signalNumBytes, 0, 64, 13
				)
				self.mySignals.v, receivedData = self.unpackBytes(
					'd', receivedData, self.mySignals.v
				)

				# Receive angular velocity (omega)
				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(
					signalNumBytes, 0, 64, 14
				)
				self.mySignals.omega, receivedData = self.unpackBytes(
					'd', receivedData, self.mySignals.omega
				)

				# ====================================================
				# SEND OUTPUTS TO CAN
				# ====================================================

				# Send x
				vsiCanPythonGateway.setCanId(10)
				vsiCanPythonGateway.setCanPayloadBits(
					self.packBytes('d', self.mySignals.x), 0, 64
				)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				# Send y
				vsiCanPythonGateway.setCanId(11)
				vsiCanPythonGateway.setCanPayloadBits(
					self.packBytes('d', self.mySignals.y), 0, 64
				)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				# Send theta
				vsiCanPythonGateway.setCanId(12)
				vsiCanPythonGateway.setCanPayloadBits(
					self.packBytes('d', self.mySignals.theta), 0, 64
				)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				# ====================================================
				# DEBUG PRINT
				# ====================================================
				print("\n+=simulator+=")
				print("  VSI time:", vsiCommonPythonApi.getSimulationTimeInNs(), "ns")

				print("  Inputs:")
				print("\tv =", self.mySignals.v)
				print("\tomega =", self.mySignals.omega)

				print("  Outputs:")
				print("\tx =", self.mySignals.x)
				print("\ty =", self.mySignals.y)
				print("\ttheta =", self.mySignals.theta)
				print("\n\n")

				# ====================================================
				# TIME SYNCHRONIZATION
				# ====================================================
				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")

				nextExpectedTime += self.simulationStep

				# Advance simulation time
				if(vsiCommonPythonApi.getSimulationTimeInNs() >= nextExpectedTime):
					continue

				if(nextExpectedTime > self.totalSimulationTime):
					remainingTime = self.totalSimulationTime - vsiCommonPythonApi.getSimulationTimeInNs()
					vsiCommonPythonApi.advanceSimulation(remainingTime)
					break

				vsiCommonPythonApi.advanceSimulation(
					nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs()
				)

		except Exception as e:
			if str(e) == "stopRequested":
				print("Terminate signal received")
				vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
			else:
				print(f"Error: {str(e)}")


	# ============================================================
	# PACK / UNPACK FUNCTIONS (BINARY SERIALIZATION)
	# ============================================================

	def packBytes(self, signalType, signal):
		# Convert Python data → binary (for CAN)
		if isinstance(signal, list):
			return struct.pack(f'={len(signal)}{signalType}', *signal)
		else:
			return struct.pack(f'={signalType}', signal)


	def unpackBytes(self, signalType, packedBytes, signal=""):
		# Convert binary → Python data
		numBytes = struct.calcsize(signalType)
		value = struct.unpack(f'={signalType}', packedBytes[:numBytes])[0]
		return value, packedBytes[numBytes:]


	# ============================================================
	# UPDATE SIMULATION PARAMETERS
	# ============================================================
	def updateInternalVariables(self):
		self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
		self.stopRequested = vsiCommonPythonApi.isStopRequested()
		self.simulationStep = vsiCommonPythonApi.getSimulationStep()


# ============================================================
# MAIN ENTRY POINT
# ============================================================
def main():
	inputArgs = argparse.ArgumentParser(" ")

	# Simulation connection args
	inputArgs.add_argument('--domain', default='AF_UNIX')
	inputArgs.add_argument('--server-url', default='localhost')

	args = inputArgs.parse_args()

	simulator = Simulator(args)
	simulator.mainThread()


if __name__ == '__main__':
    main()