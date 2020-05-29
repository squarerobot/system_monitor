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
        first_interface = True
        for item in status.values:
            key = item.key
            value = item.value
            if key == 'Update Status':
                continue
            if key == 'Time Since Update':
                net_status.time = float(value)
                continue
            if key == 'Interface Name':
                if first_interface:
                    first_interface = False
                else:
                    net_status.interfaces.append(inter)
                inter = Interface()
                inter.name = value
                continue
            if inter.name not in key:
                continue
            key_pattern = inter.name + ' State'
            if key_pattern == key:
                inter.state = value
                continue
            key_pattern = inter.name + ' Input Traffic'
            if key_pattern == key:
                value = value.split(' ')
                inter.input = float(value[0])
                continue
            key_pattern = inter.name + ' Output Traffic'
            if key_pattern == key:
                value = value.split(' ')
                inter.output = float(value[0])
                continue
            key_pattern = inter.name + ' MTU'
            if key_pattern == key:
                inter.mtu = int(value)
                continue
            key_pattern = inter.name + ' Total received MB'
            if key_pattern == key:
                inter.received = float(value)
                continue
            key_pattern = inter.name + ' Total transmitted MB'
            if key_pattern == key:
                inter.transmitted = float(value)
                continue
            key_pattern = inter.name + ' Collisions'
            if key_pattern == key:
                inter.transmitted = int(value)
                continue
            key_pattern = inter.name + ' Rx Errors'
            if key_pattern == key:
                inter.rxError = int(value)
                continue
            key_pattern = inter.name + ' Tx Errors'
            if key_pattern == key:
                inter.txError = int(value)
                continue
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
            total_item = False
            used_item = False
            free_item = False
            key = item.key
            value = item.value
            if key in ('Memory Status', 'Update Status'):
                continue
            if key == 'Time Since Update':
                mem_status.time = float(value)
                continue
            if '(Physical)' in key:
                phy_item = True
            if '(Swap)' in key:
                swap_item = True
            if '(Physical w/o Buffers)' in key:
                buf_item = True
            if 'Total Memory'in key:
                total_item = True
            if 'Used Memory' in key:
                used_item = True
            if 'Free Memory' in key:
                free_item = True

            value = value.split("M")
            value = int(value[0])

            if used_item:
                if phy_item:
                    phy_mem.used = value
                elif swap_item:
                    swp_mem.used = value
                elif buf_item:
                    buf_mem.used = value
                else:
                    mem_status.usedM = value

            if total_item:
                if phy_item:
                    phy_mem.total = value
                elif swap_item:
                    swp_mem.total = value
                elif buf_item:
                    buf_mem.total = value
                else:
                    mem_status.totalM = value

            if free_item:
                if phy_item:
                    phy_mem.free = value
                elif swap_item:
                    swp_mem.free = value
                elif buf_item:
                    buf_mem.free = value
                else:
                    mem_status.freeM = value


        mem_status.memories.append(phy_mem)
        mem_status.memories.append(swp_mem)
        mem_status.memories.append(buf_mem)
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
                continue
            if key == 'Time Since Update':
                aux_usa.time = float(value)
                continue
            if key == 'Logical Core Number':
                aux_usa.logical_cores = int(value)
                continue
            if key == 'Load Average Status':
                aux_usa.load_status = value
                continue
            if key == 'Load Average (1min)':
                value = value.split('%')
                aux_usa.load_avg1 = float(value[0])
                continue
            if key == 'Load Average (5min)':
                value = value.split('%')
                aux_usa.load_avg5 = float(value[0])
                continue
            if key == 'Load Average (15min)':
                value = value.split('%')
                aux_usa.load_avg15 = float(value[0])
                continue
            if key == 'Load Now':
                value = value.split('%')
                aux_usa.load_now = float(value[0])
                continue
            if key == 'CPU Clock Speed':
                value = value.split('MHz')
                aux_usa.speed = float(value[0])
                continue

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
        first_disk = True
        for item in status.values:
            key = item.key
            value = item.value
            if key == 'Update Status':
                aux_stat.status = value
                continue
            if key == 'Time Since Update':
                aux_stat.time = float(value)
                continue
            if key == 'Disk Space Reading':
                aux_stat.space_reading = value
                continue
            if 'Name' in key:
                aux = key.split(' ')
                current_disk = aux[1]
                if first_disk:
                    first_disk = False
                else:
                    aux_stat.disks.append(disk)
                disk = Disk()
                disk.id = int(current_disk)
                disk_name_key = 'Disk ' + str(current_disk) + ' Name'
                disk_size_key = 'Disk ' + str(current_disk) + ' Size'
                disk_aval_key = 'Disk ' + str(current_disk) + ' Available'
                disk_use_key = 'Disk ' + str(current_disk) + ' Use'
                disk_stat_key = 'Disk ' + str(current_disk) + ' Status'
                disk_mp_key = 'Disk ' + str(current_disk) + ' Mount Point'
            if key == disk_name_key:
                disk.name = value
                continue
            if key == disk_size_key:
                value = value.split('G')
                disk.size = float(value[0])
                continue
            if key == disk_aval_key:
                value = value.split('G')
                disk.available = float(value[0])
                continue
            if key == disk_use_key:
                value = value.split('%')
                disk.use = float(value[0])
                continue
            if key == disk_stat_key:
                disk.status = value
                continue
            if key == disk_mp_key:
                disk.mount_point = value
                continue

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
