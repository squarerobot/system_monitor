#! /usr/bin/env python

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from system_monitor.msg import *

class Monitor():

    def __init__(self):
        self._pub = rospy.Publisher('~diagnostics', Diagnostic, queue_size=1)
        self._diag_net = DiagnosticNET()
        self._diag_mem = DiagnosticMEM()
        self._diag_cpu_temp = DiagnosticCPUTemperature()
        self._diag_cpu_usa = DiagnosticCPUUsage()
        self._diag_hdd = DiagnosticHDD()
        r = rospy.get_param("rate_param", 5)
        self._rate = rospy.Rate(r)

    #Update network values
    def update_net_values(self, status):
        self._diag_net.name = status.name
        self._diag_net.message = status.message
        self._diag_net.hardware_id = status.hardware_id
        net_status = NetStatus()
        net_status.status = status.values[0].value
        net_status.time = float(status.values[1].value)
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
            net_status.interfaces.append(inter)
        self._diag_net.status = net_status
        self.publish_info()

    #Update memory values
    def update_mem_values(self, status):
        self._diag_mem.name = status.name
        self._diag_mem.message = status.message
        self._diag_mem.hardware_id = status.hardware_id
        mem_status = MEMStatus()
        phy_mem = Memory()
        swp_mem = Memory()
        buf_mem = Memory()
        phy_mem.name = "Physical"
        swp_mem.name = "Swap"
        buf_mem.name = "Physical w/o buffers"
        for item in status.values:
            phy_item = False
            swap_item = False
            buf_item = False
            key = item.key
            value = item.value
            if key == 'Time Since Last Update':
                mem_status.time = float(value)
            if '(Physical)' in key:
                phy_item = True
            if '(Swap)' in key:
                swap_item = True
            if '(Physical w/o Buffers)' in key:
                buf_item = True
            if 'Total Memory'in key:
                value = value.split("M")
                if phy_item:
                    phy_mem.total = int(value[0])
                elif swap_item:
                    swp_mem.total = int(value[0])
                elif buf_item:
                    buf_mem.total = int(value[0])
                else:
                    mem_status.totalM = int(value[0])
            if 'Used Memory' in key:
                value = value.split("M")
                if phy_item:
                    phy_mem.used = int(value[0])
                elif swap_item:
                    swp_mem.used = int(value[0])
                elif buf_item:
                    buf_mem.used = int(value[0])
                else:
                    mem_status.usedM = int(value[0])

            if 'Free Memory' in key:
                value = value.split("M")
                if phy_item:
                    phy_mem.free = int(value[0])
                elif swap_item:
                    swp_mem.free = int(value[0])
                elif buf_item:
                    buf_mem.free = int(value[0])
                else:
                    mem_status.freeM = int(value[0])

        mem_status.memories.append(phy_mem)
        mem_status.memories.append(swp_mem)
        mem_status.memories.append(buf_mem)
        # mem_status.time = float(status.values[1].value)
        # mem_status.totalM = int(status.values[-3].value[:-1])
        # mem_status.usedM = int(status.values[-2].value[:-1])
        # mem_status.freeM = int(status.values[-1].value[:-1])
        # names = ['Physical','Swap']
        # for i in xrange(0, 2):
        #     mem = Memory()
        #     mem.name = names[i]
        #     mem.total = int(status.values[3+5*i].value[:-1])
        #     mem.used = int(status.values[4+5*i].value[:-1])
        #     mem.free = int(status.values[5+5*i].value[:-1])
        #     mem_status.memories.append(mem)
        # mem = Memory()
        # mem.name = "Physical w/o buffers"
        # mem.used = int(status.values[6].value[:-1])
        # mem.free = int(status.values[7].value[:-1])
        # mem_status.memories.append(mem)
        self._diag_mem.status = mem_status
        self.publish_info()

    #Update cpu_temp values
    def update_cpu_temp_values(self, status):
        self._diag_cpu_temp.name = status.name
        self._diag_cpu_temp.message = status.message
        self._diag_cpu_temp.hardware_id = status.hardware_id
        aux_temp = CPUTemperatureStatus()
        aux_temp.status = status.values[0].value
        aux_temp.time = float(status.values[1].value)
        aux_temp.global_temp = float(status.values[2].value)
        for i in range(3, len(status.values)):
            core = CoreTemp()
            core.id = i - 3
            core.temp = float(status.values[i].value)
            aux_temp.cores.append(core)
        self._diag_cpu_temp.status = aux_temp
        self.publish_info()

    #Update cpu_usage values
    def update_cpu_usa_values(self, status):
        self._diag_cpu_usa.name = status.name
        self._diag_cpu_usa.message = status.message
        self._diag_cpu_usa.hardware_id = status.hardware_id
        aux_usa = CPUUsageStatus()
        for item in status.values:
            key = item.key
            value = item.value
            if key == 'Update Status':
                aux_usa.status = value
            if key == 'Time Since Last Update':
                aux_usa.time = float(value)
            if key == 'Logical Core Number':
                aux_usa.logical_cores = int(value)
            if key == 'Load Average Status':
                aux_usa.load_status = value
            if key == 'Load Average (1min)':
                value = value.split('%')
                aux_usa.load_avg1 = float(value[0])
            if key == 'Load Average (5min)':
                value = value.split('%')
                aux_usa.load_avg5 = float(value[0])
            if key == 'Load Average (15min)':
                value = value.split('%')
                aux_usa.load_avg15 = float(value[0])
            if key == 'Load Now':
                value = value.split('%')
                aux_usa.load_now = float(value[0])
            if key == 'CPU Clock Speed':
                value = value.split('MHz')
                aux_usa.speed = float(value[0])

        usa_freq_core_labels = []
        usa_stat_core_labels = []
        usa_user_core_labels = []
        usa_nice_core_labels = []
        usa_idle_core_labels = []
        usa_sys_core_labels = []
        for i in range(0, aux_usa.logical_cores):
            core_freq_str = 'Core '
            core_freq_str += str(i)
            core_freq_str += ' Clock Speed'
            usa_freq_core_labels.append(core_freq_str)
            core_stat_str = 'Core '
            core_stat_str += str(i)
            core_stat_str += ' Status'
            usa_stat_core_labels.append(core_stat_str)
            core_user_str = 'Core '
            core_user_str += str(i)
            core_user_str += ' User'
            usa_user_core_labels.append(core_user_str)
            core_nice_str = 'Core '
            core_nice_str += str(i)
            core_nice_str += ' Nice'
            usa_nice_core_labels.append(core_nice_str)
            core_sys_str = 'Core '
            core_sys_str += str(i)
            core_sys_str += ' System'
            usa_sys_core_labels.append(core_sys_str)
            core_idle_str = 'Core '
            core_idle_str += str(i)
            core_idle_str += ' Idle'
            usa_idle_core_labels.append(core_idle_str)
            core = CoreUsage()
            aux_usa.cores.append(core)

        current_core = 0
        for item in status.values:
            key = item.key
            value = item.value
            if key == usa_freq_core_labels[current_core]:
                aux_usa.cores[current_core].id = current_core
                value = value.split('MHz')
                aux_usa.cores[current_core].speed = float(value[0])
                current_core += 1
                if current_core >= aux_usa.logical_cores:
                    break

        current_core = 0
        for item in status.values:
            key = item.key
            value = item.value
            if key == usa_stat_core_labels[current_core]:
                aux_usa.cores[current_core].status = value
            if key == usa_user_core_labels[current_core]:
                value = value.split('%')
                aux_usa.cores[current_core].user = float(value[0])
            if key == usa_nice_core_labels[current_core]:
                value = value.split('%')
                aux_usa.cores[current_core].nice = float(value[0])
            if key == usa_sys_core_labels[current_core]:
                value = value.split('%')
                aux_usa.cores[current_core].system = float(value[0])
            if key == usa_idle_core_labels[current_core]:
                value = value.split('%')
                aux_usa.cores[current_core].idle = float(value[0])
                current_core += 1
                if current_core >= aux_usa.logical_cores:
                    break
        self._diag_cpu_usa.status = aux_usa
        self.publish_info()

    #Update hdd values
    def update_hdd_values(self, status):
        self._diag_hdd.name = status.name
        self._diag_hdd.message = status.message
        self._diag_hdd.hardware_id = status.hardware_id
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
        self.publish_info()

    #Publish info
    def publish_info(self):
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
        mon.update_mem_values(data.status[0])
    elif data.status[0].name.startswith('Network'):
        #Extract useful data from network
        mon.update_net_values(data.status[0])
    elif data.status[0].name.startswith('CPU Temperature'):
        #Extract useful data from cpu
        mon.update_cpu_temp_values(data.status[0])
        mon.update_cpu_usa_values(data.status[1])
    elif data.status[0].name.startswith("HDD Usage"):
        #Extract useful data from disk
        mon.update_hdd_values(data.status[0])


if __name__ == '__main__':
    rospy.init_node('system_monitor_node')
    mon = Monitor()
    rospy.Subscriber('/diagnostics', DiagnosticArray, callback)
    rospy.spin()
