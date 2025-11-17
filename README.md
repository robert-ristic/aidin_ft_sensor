# ROS 2 Driver for Aidin Robotics 6-Axis Force-Torque Sensor

This repository provides a simple **ROS 2 interface** for the **Aidin Robotics 6-Axis Force-Torque Sensor (AFT200-D80-EC)** over **EtherCAT**. It allows you to read force and torque measurements in real time and publish them as ROS 2 `geometry_msgs/msg/WrenchStamped` messages.

---

## **Requirements**

- **SOEM (Simple Open EtherCAT Master)**
    - GitHub: [https://github.com/OpenEtherCATsociety/SOEM](https://github.com/OpenEtherCATsociety/SOEM)
    - Tested with version 2.0.0.
    - **Install system-wide** using:

      ```bash
      cmake -DCMAKE_INSTALL_PREFIX=/usr ..
      make
      sudo make install
      ```

- **Ubuntu / ROS 2 workspace**  
  Tested on Ubuntu 24.04 / ROS 2 Jazzy, but should work with newer ROS 2 versions.

---

## **Build the Package**

Inside your ROS 2 workspace:

```bash
colcon build --packages-select aidin_ft_sensor --symlink-install
```

## Set Permissions for EtherCAT

EtherCAT requires access to raw network sockets. Before running the node, grant the necessary capabilities:
```bash
sudo setcap cap_net_raw,cap_net_admin+ep "$(readlink -f $(ros2 pkg prefix aidin_ft_sensor)/lib/aidin_ft_sensor/aidin_ft_sensor_node)"
```

## Run the Node
You can run the node and configure the network interface and update rate via ROS 2 parameters:
```bash
ros2 run aidin_ft_sensor aidin_ft_sensor_node --ros-args -p ifname:=enxd0c0bf2f676d -p update_rate:=200
```
Parameters Parameter     | Default           | Description                               |
|---------------|-----------------|-------------------------------------------|
| ifname        | enxd0c0bf2f676d | Ethernet interface connected to the sensor |
| update_rate   | 100             | Node publishing rate in Hz                 |

Example above sets the interface to enxd0c0bf2f676d and updates at 200 Hz.

## Output
The node publishes ```geometry_msgs/msg/WrenchStamped``` messages on the ```wrench``` topic. Each message contains:
* ```wrench.force.x, y, z``` → forces in Newtons
* ```wrench.torque.x, y, z``` → torques in Newton-meters