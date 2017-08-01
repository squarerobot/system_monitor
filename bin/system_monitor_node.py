#! /usr/bin/env python

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from system_monitor.msg import *

class Monitor():

	def __init__(self, r):
		self._pub = rospy.Publisher('/robotnikDiagnostic', Diagnostic, queue_size=1)
		self._diag_net = DiagnosticNET()
		self._diag_mem = DiagnosticMEM()
		self._diag_cpu_temp = DiagnosticCPUTemperature()
		self._diag_cpu_usa = DiagnosticCPUUsage()
		self._diag_hdd = DiagnosticHDD()
		self._rate = rospy.Rate(r)

	#Update network values
	def updateNetValues(self, status):
		#TODO: Modify current structure of message
		self._diag_net.name = status.name
		self._diag_net.message = status.message
		self._diag_net.interface[:] = []
		ifaces = (len(status.values) - 2) / 10
		for i in xrange(0, ifaces):
			inter = Interface()
			inter.name = status.values[2+10*i].value
			inter.state = status.values[3+10*i].value
			inter.input = float(status.values[4+10*i].value[:-6])
			inter.output = float(status.values[5+10*i].value[:-6])
			inter.mtu = int(status.values[6+10*i].value)
			inter.received = float(status.values[7+10*i].value)
			inter.transmitted = float(status.values[8+10*i].value)
			inter.collisions = int(status.values[9+10*i].value)
			inter.rxError = int(status.values[10+10*i].value)
			inter.txError = int(status.values[11+10*i].value)
			self._diag_net.interface.append(inter)
		self.publishInfo()

	#Update memory values
	def updateMemValues(self, status):
		self._diag_mem.name = status.name
		self._diag_mem.message = status.message
		mem_status = MEMStatus()
		mem_status.time = float(status.values[1].value)
		mem_status.totalM = int(status.values[-3].value[:-1])
		mem_status.usedM = int(status.values[-2].value[:-1])
		mem_status.freeM = int(status.values[-1].value[:-1])
		names = ['Physical','Physical w/o','Swap']
		for i in xrange(0, 3):
			mem = Memory()
			mem.name = names[i]
			#TODO: check index
			mem.total = int(status.values[3+3*i].value[:-1])
			mem.used = int(status.values[4+3*i].value[:-1])
			mem.free = int(status.values[5+3*i].value[:-1])
			mem_status.memories.append(mem)
		self._diag_mem.status = mem_status
		self.publishInfo()

	#Update cpu_temp values
	def updateCpuTempValues(self, status):
		self._diag_cpu_temp.name = status.name
		self._diag_cpu_temp.message = status.message
		aux_temp = CPUTemperatureStatus()
		aux_temp.status = status.values[0].value
		aux_temp.time = float(status.values[1].value)
		for i in range(2, len(status.values)):
			core = CoreTemp()
			core.id = i - 2
			core.temp = float(status.values[i].value[:-4])
			aux_temp.cores.append(core)
		self._diag_cpu_temp.status = aux_temp
		self.publishInfo()

	#Update cpu_usage values
	def updateCpuUsaValues(self, status):
		self._diag_cpu_usa.name = status.name
		self._diag_cpu_usa.message = status.message
		aux_usa = CPUUsageStatus()
		len_values = len(status.values)
		num_cores = (len_values - 6)/6
		aux_usa.status = status.values[0].value
		aux_usa.time = float(status.values[1].value)
		aux_usa.load_status = status.values[len_values - 4].value
		aux_usa.load_avg1 = float(status.values[len_values - 3].value[:-1])
		aux_usa.load_avg5 = float(status.values[len_values - 2].value[:-1])
		aux_usa.load_avg15 = float(status.values[len_values - 1].value[:-1])
		for i in range(0, num_cores):
			core = CoreUsage()
			core.id = i
			core.speed = float(status.values[i +2].value[:-3])
			core.status = status.values[2 + num_cores + 5*i].value
			core.system = float(status.values[3 + num_cores + 5*i].value[:-1])
			core.user = float(status.values[4 + num_cores + 5*i].value[:-1])
			core.nice = float(status.values[5 + num_cores + 5*i].value[:-1])
			core.idle = float(status.values[6 + num_cores + 5*i].value[:-1].replace(",","."))
			aux_usa.cores.append(core)
		self._diag_cpu_usa.status = aux_usa
		self.publishInfo()

	#Update hdd values
	def updateHddValues(self, status):
		self._diag_hdd.name = status.name
		self._diag_hdd.message = status.message
		aux_stat = HDDStatus()
		aux_stat.status = status.values[0].value
		aux_stat.time = float(status.values[1].value)
		aux_stat.space_reading = status.values[2].value
		num_disks = (len(status.values) - 3)/6
		for i in range(0,num_disks):
			disk = Disk()
			disk.id = i + 1
			disk.name = status.values[3 + i * 6].value
			disk.size = float(status.values[4 + i * 6].value[:-1])
			disk.available = float(status.values[5 + i * 6].value[:-1])
			disk.use = float(status.values[6 + i * 6].value[:-1])
			disk.status = status.values[7 + i * 6].value
			disk.mount_point = status.values[8 + i * 6].value
			aux_stat.disks.append(disk)
		self._diag_hdd.status = aux_stat
		self.publishInfo()

	#Publish info
	def publishInfo(self):
		msg = Diagnostic()
		msg.diagNet = self._diag_net
		msg.diagMem = self._diag_mem
		msg.diagCpuTemp = self._diag_cpu_temp
		msg.diagCpuUsage = self._diag_cpu_usa
		msg.diagHdd = self._diag_hdd
		self._rate.sleep()
		self._pub.publish(msg)


# Print CPU status
def callback(data):
	if data.status[0].name.startswith('Memory'):
		#Extract useful data from memory
		mon.updateMemValues(data.status[0])
	elif data.status[0].name.startswith('Network'):
		#Extract useful data from network
		mon.updateNetValues(data.status[0])
	elif data.status[0].name.startswith('CPU Temperature'):
		#Extract useful data from cpu
		mon.updateCpuTempValues(data.status[0])
		mon.updateCpuUsaValues(data.status[1])
	elif data.status[0].name.startswith("HDD Usage"):
		#Extract useful data from disk
		mon.updateHddValues(data.status[0])


if __name__ == '__main__':
	rospy.init_node('monitor_node')
	mon = Monitor(0.5)
	rospy.Subscriber('/diagnostics', DiagnosticArray, callback)
	rospy.spin()
