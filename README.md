# system_monitor

## Synopsis

System monitoring tools for ROS.

**Author(s):** Willow Garage, Inc., Jerome Maye, Ralf Kaestner

**Maintainer:** Ralf Kaestner <ralf.kaestner@gmail.com>

**License:** BSD License (BSD)

**Operating system(s):** Debian-based Linux

## Description

This project provides system monitoring tools for ROS in the form of the
following ROS nodes:

* CPU monitor
* HDD monitor
* Memory monitor
* Network monitor
* NTP monitor

Each node publishes ROS diagnostics which can conveniently be visualized
in the runtime monitor.

In this branch uses pure python functions, no external system packages are required.
## Installation

Download the repository in the src folder of your worksapce:

```bash
git clone https://github.com/RobotnikAutomation/system_monitor
git checkout psutil
```

### Requirements
* diagnostic_msgs
* psutil >= 5.7.0
* ntplib >= 0.3.4


### Installl required packages
```bash
sudo pip2 install --upgrade pip
sudo pip2 install -r requirements.txt
```

## Usage
Use the launch file in order to run the node:

```bash
roslaunch system_monitor system_monitor.launch
```

### Optional arguments

| Argument       | Environment variable | Meaning                     | Default value |
| -------------- | -------------------- | --------------------------- | ------------- |
| `machine_name` | `HOSTNAME`           | Machine name                | `localhost`   |
| `launch_cpu`   | `SYS_MON_LAUNCH_CPU` | Launch the CPU monitor node | `true`        |
| `launch_hdd`   | `SYS_MON_LAUNCH_HDD` | Launch the Hard Drive monitor node | `true`        |
| `launch_mem`   | `SYS_MON_LAUNCH_MEM` | Launch the RAM memory monitor node | `true`        |
| `launch_ntp`   | `SYS_MON_LAUNCH_NTP` | Launch the NTP monitor node | `true`        |
| `launch_net`   | `SYS_MON_LAUNCH_NET` | Launch the Network monitor node | `true`        |

