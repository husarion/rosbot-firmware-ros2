# rosbot-firmware-ros2 [deprecated]

> use https://github.com/husarion/rosbot-stm32-firmware instead

The **ROS2** compatible (dashing), **ROSbot** mobile platform's MCU firmware. Written in C/C++ using arm's **Mbed OS** framework ( mbed-os-5.14.1). DDS layer uses [eProsima Micro-XRCE-DDS Client](https://micro-xrce-dds.readthedocs.io/en/latest/index.html) library.


```
______  _____  _____  _             _                        _____ 
| ___ \|  _  |/  ___|| |           | |                      / __  \
| |_/ /| | | |\ `--. | |__    ___  | |_     _ __  ___   ___ `' / /'
|    / | | | | `--. \| '_ \  / _ \ | __|   | '__|/ _ \ / __|  / /  
| |\ \ \ \_/ //\__/ /| |_) || (_) || |_    | |  | (_) |\__ \./ /___
\_| \_| \___/ \____/ |_.__/  \___/  \__|   |_|   \___/ |___/\_____/
                                                                   
```
**firmware version:** `0.2.0`
**Status:** `In development`

## Micro-XRCE-DDS Agent installation

```bash
$ git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
$ mkdir build && cd build
$ source /opt/ros/dashing/setup.bash # to share libraries with ros2
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig /usr/local/lib/
```

## DDS communication

To start communication run `Micro-XRCE-DDS-Agent`:

```bash
$ MicroXRCEAgent serial --dev <SBC_port_name> -b <port_baudrate>
```

`<SBC_port_name>`:
- `/dev/ttyS1` for Asus Tinker Board,
- `/dev/ttyS4` for UpBoard

`<port_baudrate>`:
- `460800` for UpBoard
- `500000` for Asus Tinker Board

## ROS2 interface

ROSbot subscribes to:

* `/cmd_vel` with message type `geometry_msgs/Twist`
* `/rosbot_time` with message type `builtin_msgs/Time` - temporarily for time synchronization

ROSbot publishes to:

* `/battery` with message type `sensor_msgs/BatteryState`
* `/odom` with message type `geometry_msgs/PoseStamped`
* `/tf` with message type `tf2_msgs/TFMessage`
