#!/usr/bin/env python3
from __future__ import print_function
import struct
import sys
import argparse
import math

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway


class MySignals:
	def __init__(self):
		# Inputs
		self.x = 0
		self.y = 0
		self.theta = 0




# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions
import matplotlib
matplotlib.use('Agg')  # no display needed, saves to file
import matplotlib.pyplot as plt

times    = []
robot_xs = []
robot_ys = []
errors   = []
# End of user custom code region. Please don't edit beyond this point.
class Visualizer:

	def __init__(self, args):
		self.componentId = 2
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50103
        
		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0
        
		self.receivedNumberOfBytes = 0
		self.receivedPayload = []
		self.mySignals = MySignals()

		# Start of user custom code region. Please apply edits only within these regions:  Constructor

		# End of user custom code region. Please don't edit beyond this point.



	def mainThread(self):
		dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
		vsiCanPythonGateway.initialize(dSession, self.componentId)
		try:
			vsiCommonPythonApi.waitForReset()

			# Start of user custom code region. Please apply edits only within these regions:  After Reset

			# End of user custom code region. Please don't edit beyond this point.
			self.updateInternalVariables()

			if(vsiCommonPythonApi.isStopRequested()):
				raise Exception("stopRequested")
			nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
			while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

				# Start of user custom code region. Please apply edits only within these regions:  Inside the while loop
				t     = vsiCommonPythonApi.getSimulationTimeInNs() * 1e-9
				x     = self.mySignals.x
				y     = self.mySignals.y

				times.append(t)
				robot_xs.append(x)
				robot_ys.append(y)
				y_ref = 2.0 * math.sin(0.3 * x)
				errors.append(abs(y - y_ref))  # lateral error = distance from curved path
				# End of user custom code region. Please don't edit beyond this point.

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 10)
				self.mySignals.x, receivedData = self.unpackBytes('d', receivedData, self.mySignals.x)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 11)
				self.mySignals.y, receivedData = self.unpackBytes('d', receivedData, self.mySignals.y)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 12)
				self.mySignals.theta, receivedData = self.unpackBytes('d', receivedData, self.mySignals.theta)

				# Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				print("\n+=visualizer+=")
				print("  VSI time:", end = " ")
				print(vsiCommonPythonApi.getSimulationTimeInNs(), end = " ")
				print("ns")
				print("  Inputs:")
				print("\tx =", end = " ")
				print(self.mySignals.x)
				print("\ty =", end = " ")
				print(self.mySignals.y)
				print("\ttheta =", end = " ")
				print(self.mySignals.theta)
				print("\n\n")

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")
				nextExpectedTime += self.simulationStep

				if(vsiCommonPythonApi.getSimulationTimeInNs() >= nextExpectedTime):
					continue

				if(nextExpectedTime > self.totalSimulationTime):
					remainingTime = self.totalSimulationTime - vsiCommonPythonApi.getSimulationTimeInNs()
					vsiCommonPythonApi.advanceSimulation(remainingTime)
					break

				vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())
		except Exception as e:
			if str(e) == "stopRequested":
				print("Terminate signal has been received from one of the VSI clients")
				# Advance time with a step that is equal to "simulationStep + 1" so that all other clients
				# receive the terminate packet before terminating this client
				vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
			else:
				print(f"An error occurred: {str(e)}")
		except:
			# Advance time with a step that is equal to "simulationStep + 1" so that all other clients
			# receive the terminate packet before terminating this client
			vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)




		# Start of user custom code region. Please apply edits only within these regions:  Protocol's callback function
		# Generate plots when simulation ends
		if len(times) > 0:
			path_x = [i * 0.1 for i in range(500)]
			path_y = [2.0 * math.sin(0.3 * px) for px in path_x]

			fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

			ax1.plot(path_x, path_y, 'b--', linewidth=2, label='Reference path')
			ax1.plot(robot_xs, robot_ys, 'r-', linewidth=1.5, label='Robot trajectory')
			ax1.set_title('Trajectory vs Path')
			ax1.set_xlabel('X (m)')
			ax1.set_ylabel('Y (m)')
			ax1.legend()
			ax1.grid(True)

			ax2.plot(times, errors, 'g-', linewidth=1.5)
			ax2.set_title('Lateral Error over Time')
			ax2.set_xlabel('Time (s)')
			ax2.set_ylabel('Error (m)')
			ax2.grid(True)

			plt.tight_layout()
			plt.savefig('E2_curved_run3.png')
			print("Plot saved!")

			# Print KPIs
			# Overshoot
			overshoot = max(errors)

			# Settling time = first time error stays below 5% of max error
			threshold = 0.1 * overshoot
			settling_time = times[-1]
			for i in range(len(errors)):
				if all(e < threshold for e in errors[i:]):
					settling_time = times[i]
					break

			# Steady state error = average of last 10% of errors
			last_10 = errors[int(0.9*len(errors)):]
			ss_error = sum(last_10) / len(last_10)

			print(f"Max error (overshoot):     {overshoot:.4f} m")
			print(f"Settling time:             {settling_time:.1f} s")
			print(f"Final error (steady state):{ss_error:.4f} m")

		# End of user custom code region. Please don't edit beyond this point.



	def packBytes(self, signalType, signal):
		if isinstance(signal, list):
			if signalType == 's':
				packedData = b''
				for str in signal:
					str += '\0'
					str = str.encode('utf-8')
					packedData += struct.pack(f'={len(str)}s', str)
				return packedData
			else:
				return struct.pack(f'={len(signal)}{signalType}', *signal)
		else:
			if signalType == 's':
				signal += '\0'
				signal = signal.encode('utf-8')
				return struct.pack(f'={len(signal)}s', signal)
			else:
				return struct.pack(f'={signalType}', signal)



	def unpackBytes(self, signalType, packedBytes, signal = ""):
		if isinstance(signal, list):
			if signalType == 's':
				unpackedStrings = [''] * len(signal)
				for i in range(len(signal)):
					nullCharacterIndex = packedBytes.find(b'\0')
					if nullCharacterIndex == -1:
						break
					unpackedString = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
					unpackedStrings[i] = unpackedString
					packedBytes = packedBytes[nullCharacterIndex + 1:]
				return unpackedStrings, packedBytes
			else:
				unpackedVariable = struct.unpack(f'={len(signal)}{signalType}', packedBytes[:len(signal)*struct.calcsize(f'={signalType}')])
				packedBytes = packedBytes[len(unpackedVariable)*struct.calcsize(f'={signalType}'):]
				return list(unpackedVariable), packedBytes
		elif signalType == 's':
			nullCharacterIndex = packedBytes.find(b'\0')
			unpackedVariable = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
			packedBytes = packedBytes[nullCharacterIndex + 1:]
			return unpackedVariable, packedBytes
		else:
			numBytes = 0
			if signalType in ['?', 'b', 'B']:
				numBytes = 1
			elif signalType in ['h', 'H']:
				numBytes = 2
			elif signalType in ['f', 'i', 'I', 'L', 'l']:
				numBytes = 4
			elif signalType in ['q', 'Q', 'd']:
				numBytes = 8
			else:
				raise Exception('received an invalid signal type in unpackBytes()')
			unpackedVariable = struct.unpack(f'={signalType}', packedBytes[0:numBytes])[0]
			packedBytes = packedBytes[numBytes:]
			return unpackedVariable, packedBytes

	def updateInternalVariables(self):
		self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
		self.stopRequested = vsiCommonPythonApi.isStopRequested()
		self.simulationStep = vsiCommonPythonApi.getSimulationStep()



def main():
	inputArgs = argparse.ArgumentParser(" ")
	inputArgs.add_argument('--domain', metavar='D', default='AF_UNIX', help='Socket domain for connection with the VSI TLM fabric server')
	inputArgs.add_argument('--server-url', metavar='CO', default='localhost', help='server URL of the VSI TLM Fabric Server')

	# Start of user custom code region. Please apply edits only within these regions:  Main method

	# End of user custom code region. Please don't edit beyond this point.

	args = inputArgs.parse_args()
                      
	visualizer = Visualizer(args)
	visualizer.mainThread()



if __name__ == '__main__':
    main()
