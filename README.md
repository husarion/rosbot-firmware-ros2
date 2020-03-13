# rosbot-firmware-ros2
The **ROS2** compatible (dashing), **ROSbot** mobile platform's MCU firmware. Written in C/C++ using arm's **Mbed OS** framework ( mbed-os-5.14.1). DDS layer uses [eProsima Micro-XRCE-DDS Client](https://micro-xrce-dds.readthedocs.io/en/latest/index.html) library.


```
______  _____  _____  _             _                        _____ 
| ___ \|  _  |/  ___|| |           | |                      / __  \
| |_/ /| | | |\ `--. | |__    ___  | |_     _ __  ___   ___ `' / /'
|    / | | | | `--. \| '_ \  / _ \ | __|   | '__|/ _ \ / __|  / /  
| |\ \ \ \_/ //\__/ /| |_) || (_) || |_    | |  | (_) |\__ \./ /___
\_| \_| \___/ \____/ |_.__/  \___/  \__|   |_|   \___/ |___/\_____/
                                                                   
```
**firmware version:** `0.1.0`

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

**Status:** In development