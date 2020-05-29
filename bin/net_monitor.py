#!/usr/bin/env python
############################################################################
#    Copyright (C) 2009, Willow Garage, Inc.                               #
#    Copyright (C) 2013 by Ralf Kaestner                                   #
#    ralf.kaestner@gmail.com                                               #
#    Copyright (C) 2013 by Jerome Maye                                     #
#    jerome.maye@mavt.ethz.ch                                              #
#                                                                          #
#    All rights reserved.                                                  #
#                                                                          #
#    Redistribution and use in source and binary forms, with or without    #
#    modification, are permitted provided that the following conditions    #
#    are met:                                                              #
#                                                                          #
#    1. Redistributions of source code must retain the above copyright     #
#       notice, this list of conditions and the following disclaimer.      #
#                                                                          #
#    2. Redistributions in binary form must reproduce the above copyright  #
#       notice, this list of conditions and the following disclaimer in    #
#       the documentation and/or other materials provided with the         #
#       distribution.                                                      #
#                                                                          #
#    3. The name of the copyright holders may be used to endorse or        #
#       promote products derived from this software without specific       #
#       prior written permission.                                          #
#                                                                          #
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   #
#    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     #
#    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     #
#    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        #
#    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  #
#    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  #
#    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      #
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      #
#    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    #
#    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN     #
#    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       #
#    POSSIBILITY OF SUCH DAMAGE.                                           #
############################################################################

from __future__ import with_statement

import rospy

import traceback
import threading
from threading import Timer
import sys, os, time
from time import sleep
import psutil
import string
import re

import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

net_level_warn = 0.95
net_capacity = 128

stat_dict = {0: 'OK', 1: 'Warning', 2: 'Error'}

def update_status_stale(stat, last_update_time):
  time_since_update = rospy.get_time() - last_update_time
  stale_status = 'OK'
  if time_since_update > 20 and time_since_update <= 35:
    stale_status = 'Lagging'
    if stat.level == DiagnosticStatus.OK:
      stat.message = stale_status
    elif stat.message.find(stale_status) < 0:
      stat.message = ', '.join([stat.message, stale_status])
    stat.level = max(stat.level, DiagnosticStatus.WARN)
  if time_since_update > 35:
    stale_status = 'Stale'
    if stat.level == DiagnosticStatus.OK:
      stat.message = stale_status
    elif stat.message.find(stale_status) < 0:
      stat.message = ', '.join([stat.message, stale_status])
    stat.level = max(stat.level, DiagnosticStatus.ERROR)
  stat.values.pop(0)
  stat.values.pop(0)
  stat.values.insert(0, KeyValue(key = 'Update Status', value = stale_status))
  stat.values.insert(1, KeyValue(key = 'Time Since Update',
    value = str(time_since_update)))

def get_sys_net_stat(iface, sys):
  cmd = 'cat /sys/class/net/%s/statistics/%s' %(iface, sys)
  p = subprocess.Popen(cmd,
                       stdout = subprocess.PIPE,
                       stderr = subprocess.PIPE, shell = True)
  stdout, stderr = p.communicate()
  return (p.returncode, stdout.strip())

def get_sys_net(iface, sys):
  cmd = 'cat /sys/class/net/%s/%s' %(iface, sys)
  p = subprocess.Popen(cmd,
                       stdout = subprocess.PIPE,
                       stderr = subprocess.PIPE, shell = True)
  stdout, stderr = p.communicate()
  return (p.returncode, stdout.strip())

class NetMonitor():
  def __init__(self, hostname, diag_hostname):
    self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size = 100)
    self._mutex = threading.Lock()
    self._net_level_warn = rospy.get_param('~net_level_warn', net_level_warn)
    self._net_capacity = rospy.get_param('~net_capacity', net_capacity)
    self._usage_timer = None
    self._usage_stat = DiagnosticStatus()
    self._usage_stat.name = 'Network Usage (%s)' % diag_hostname
    self._usage_stat.level = 1
    self._usage_stat.hardware_id = hostname
    self._usage_stat.message = 'No Data'
    self._usage_stat.values = [KeyValue(key = 'Update Status',
                               value = 'No Data' ),
                               KeyValue(key = 'Time Since Last Update',
                               value = 'N/A') ]
    self._last_usage_time = 0
    self._last_publish_time = 0
    self.check_usage()

  def cancel_timers(self):
    if self._usage_timer:
      self._usage_timer.cancel()

  def check_network(self):
    values = []
    net_dict = {0: 'OK', 1: 'High Network Usage', 2: 'Network Down', 3: 'Call Error'}
    one_iface_is_up = False
    try:
      prev_net_io_counters = psutil.net_io_counters(pernic=True)
      time_prev = time.time()
      time.sleep(1.00)
      net_io_counters = psutil.net_io_counters(pernic=True)
      time_now = time.time()
      net_stats = psutil.net_if_stats()
      for iface in net_io_counters:
        iface_name = iface
        if iface_name == "lo":
            continue
        isup = net_stats[iface].isup


        iface_up_bytes_prv = prev_net_io_counters[iface].bytes_sent
        iface_up_bytes_now = net_io_counters[iface].bytes_sent
        iface_upload_speed = iface_up_bytes_now - iface_up_bytes_prv
        iface_upload_speed /= (time_now - time_prev)
        iface_upload_speed /= 1048576
        iface_upload_speed = round(iface_upload_speed, 2)

        iface_down_bytes_now = net_io_counters[iface].bytes_recv
        iface_down_bytes_prv = prev_net_io_counters[iface].bytes_recv
        iface_down_speed = iface_down_bytes_now - iface_down_bytes_prv
        iface_down_speed /= (time_now - time_prev)
        iface_down_speed /= 1048576
        iface_down_speed = round(iface_down_speed, 2)

        iface_mtu = net_stats[iface].mtu
        iface_total_rx = net_io_counters[iface].bytes_recv
        iface_total_rx /= 1048576
        iface_total_tx = net_io_counters[iface].bytes_sent
        iface_total_tx /= 1048576
        iface_error_in = net_io_counters[iface].errin
        iface_error_out = net_io_counters[iface].errout
        collisions_path = "/sys/class/net/"
        collisions_path += iface
        collisions_path += "/statistics/collisions"

        col_file = open(collisions_path, "r")
        iface_col = int(col_file.read())
        col_file.close()

        if isup:
            iface_state = "up"
            one_iface_is_up = True
            level = DiagnosticStatus.OK

        net_usage_in = iface_upload_speed / self._net_capacity
        net_usage_out = iface_down_speed / self._net_capacity

        if net_usage_in > self._net_level_warn or\
          net_usage_out > self._net_level_warn:
          level = DiagnosticStatus.WARN

        if not isup:
            iface_state = "down"
            if not one_iface_is_up:
              level = DiagnosticStatus.ERROR

        values.append(KeyValue(
          key = 'Interface Name',
          value = iface_name
        ))
        values.append(KeyValue(
          key = iface_name + ' State',
          value = iface_state
        ))
        values.append(KeyValue(
          key = iface_name + ' Input Traffic',
          value = str(iface_upload_speed) + " (MB/s)"
        ))
        values.append(KeyValue(
          key = iface_name + ' Output Traffic',
          value = str(iface_down_speed) + " (MB/s)"
        ))
        values.append(KeyValue(
          key = iface_name + ' MTU',
          value = str(iface_mtu)
        ))
        values.append(KeyValue(
          key = iface_name + ' Total received MB',
          value = str(iface_total_rx)
        ))
        values.append(KeyValue(
          key = iface_name + ' Total transmitted MB',
          value = str(iface_total_tx)
        ))
        values.append(KeyValue(
          key = iface_name + ' Collisions',
          value = str(iface_col)
        ))
        values.append(KeyValue(
          key = iface_name + ' Rx Errors',
          value = str(iface_error_in)
        ))
        values.append(KeyValue(
          key = iface_name + ' Tx Errors',
          value = str(iface_error_out)
        ))
    except Exception, e:
      rospy.logerr(traceback.format_exc())
      msg = 'Network Usage Check Error'
      values.append(KeyValue(key = msg, value = str(e)))
      level = DiagnosticStatus.ERROR
    return level, net_dict[level], values

  def check_usage(self):
    if rospy.is_shutdown():
      with self._mutex:
        self.cancel_timers()
      return
    diag_level = 0
    diag_vals = [KeyValue(key = 'Update Status', value = 'OK'),
                 KeyValue(key = 'Time Since Last Update', value = 0)]
    diag_msgs = []
    net_level, net_msg, net_vals = self.check_network()
    diag_vals.extend(net_vals)
    if net_level > 0:
      diag_msgs.append(net_msg)
    diag_level = max(diag_level, net_level)
    if diag_msgs and diag_level > 0:
      usage_msg = ', '.join(set(diag_msgs))
    else:
      usage_msg = stat_dict[diag_level]
    with self._mutex:
      self._last_usage_time = rospy.get_time()
      self._usage_stat.level = diag_level
      self._usage_stat.values = diag_vals
      self._usage_stat.message = usage_msg
      if not rospy.is_shutdown():
        self._usage_timer = threading.Timer(5.0, self.check_usage)
        self._usage_timer.start()
      else:
        self.cancel_timers()

  def publish_stats(self):
    with self._mutex:
      update_status_stale(self._usage_stat, self._last_usage_time)
      msg = DiagnosticArray()
      msg.header.stamp = rospy.get_rostime()
      msg.status.append(self._usage_stat)
      if rospy.get_time() - self._last_publish_time > 0.5:
        self._diag_pub.publish(msg)
        self._last_publish_time = rospy.get_time()

if __name__ == '__main__':
  hostname = socket.gethostname()
  hostname = hostname.replace('-', '_')

  import optparse
  parser =\
    optparse.OptionParser(
    usage="usage: net_monitor.py [--diag-hostname=cX]")
  parser.add_option("--diag-hostname", dest="diag_hostname",
                    help="Computer name in diagnostics output (ex: 'c1')",
                    metavar="DIAG_HOSTNAME",
                    action="store", default = hostname)
  options, args = parser.parse_args(rospy.myargv())
  try:
    rospy.init_node('net_monitor_%s' % hostname)
  except rospy.exceptions.ROSInitException:
    print >> sys.stderr,\
      'Network monitor is unable to initialize node. Master may not be running.'
    sys.exit(0)
  net_node = NetMonitor(hostname, options.diag_hostname)
  rate = rospy.Rate(1.0)
  try:
    while not rospy.is_shutdown():
      rate.sleep()
      net_node.publish_stats()
  except KeyboardInterrupt:
    pass
  except Exception, e:
    traceback.print_exc()
    rospy.logerr(traceback.format_exc())
  net_node.cancel_timers()
  sys.exit(0)
