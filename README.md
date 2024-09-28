# LiDAR Library for Arduino
Laser distance scan sensor (LIDAR) Arduino wrapper/controller for [kaia.ai](https://kaia.ai) home robot software platform.
Please check out the [Awesome 2D LiDARs list](https://github.com/kaiaai/awesome-2d-lidars)
and read this [blog post](https://kaia.ai/blog/arduino-lidar-library/) for more details.

Please visit the [Support Forum](https://github.com/makerspet/support/discussions/)!

This library supports:
- YDLIDAR X4, X4-PRO, X3, X3-PRO, X2/X2L, SCL models
- SLAMTEC RPLIDAR A1
- Neato XV11/Botvac
- Xiaomi Roborock Mi 1st gen LDS02RR
- 3irobotix Delta-2A, Delta-2B, Delta-2G, Delta-2A 115000 baud
  - Note: there is a fixed pattern noise issue that needs debug
- LDROBOT LD14P
- CAMSENSE X1

Other models are in the works.

![LDS_collection_labeled_v3](https://github.com/kaiaai/LDS/assets/33589365/7b6a6a6f-a27c-45de-99c9-9e60d17b4d14)

### Video: YDLIDAR SCL runs on Arduino, ROS2
<a href="http://www.youtube.com/watch?feature=player_embedded&v=7mj5hnlwmx4" target="_blank">
 <img src="http://img.youtube.com/vi/7mj5hnlwmx4/maxresdefault.jpg" alt="YDLIDAR SCL laser distance scan sensor runs on Arduino, ROS2" width="720" height="405" border="10" />
</a>

### Video: Neato XV11 runs on Arduino, ROS2
<a href="http://www.youtube.com/watch?feature=player_embedded&v=kfk1Q0RSJpI" target="_blank">
 <img src="http://img.youtube.com/vi/kfk1Q0RSJpI/maxresdefault.jpg" alt="Neato XV11 laser distance scan sensor runs on Arduino, ROS2" width="720" height="405" border="10" />
</a>

### Video: Xiaomi Mi 1st gen LDS02RR runs on Arduino, ROS2
<a href="http://www.youtube.com/watch?feature=player_embedded&v=gaDnZ4Msw0E" target="_blank">
 <img src="http://img.youtube.com/vi/gaDnZ4Msw0E/maxresdefault.jpg" alt="LDS02RR laser distance scan sensor runs on Arduino, ROS2" width="720" height="405" border="10" />
</a>

### Video: SLAMTEC RPLIDAR A1 runs on Arduino, ROS2
<a href="http://www.youtube.com/watch?feature=player_embedded&v=f8IYjfiXsMk" target="_blank">
 <img src="http://img.youtube.com/vi/f8IYjfiXsMk/maxresdefault.jpg" alt="RPLIDAR A1 laser distance scan sensor runs on Arduino, ROS2" width="720" height="405" border="10" />
</a>

### Video: YDLIDAR X3 PRO runs on Arduino, ROS2
<a href="http://www.youtube.com/watch?feature=player_embedded&v=_VuRCiO55gA" target="_blank">
 <img src="http://img.youtube.com/vi/_VuRCiO55gA/maxresdefault.jpg" alt="YDLIDAR X3 PRO laser distance scan sensor runs on Arduino, ROS2" width="720" height="405" border="10" />
</a>

### Video: LDROBOT LD14P runs on Arduino, ROS2
<a href="http://www.youtube.com/watch?feature=player_embedded&v=ebbHqs4lW0U" target="_blank">
 <img src="http://img.youtube.com/vi/ebbHqs4lW0U/maxresdefault.jpg" alt="LDROBOT LD14P LiDAR connected to Arduino, ROS2" width="720" height="405" border="10" />
</a>

### Video: CAMSENSE X1 runs on Arduino, ROS2
<a href="http://www.youtube.com/watch?feature=player_embedded&v=kNBraxYKvvI" target="_blank">
 <img src="http://img.youtube.com/vi/kNBraxYKvvI/maxresdefault.jpg" alt="CAMSENSE X1 LiDAR connected to Arduino, ROS2" width="720" height="405" border="10" />
</a>

### Video: Delta-2G runs on Arduino, ROS2
<a href="http://www.youtube.com/watch?feature=player_embedded&v=eyn5ccR_I1Y" target="_blank">
 <img src="http://img.youtube.com/vi/eyn5ccR_I1Y/maxresdefault.jpg" alt="CAMSENSE X1 LiDAR connected to Arduino, ROS2" width="720" height="405" border="10" />
</a>

## Connecting to Arduino
Please follow [this tutorial](https://kaia.ai/blog/tutorial-connect-ld14p-lidar/) to connect LDROBOT LD14P to Arduino ESP32.

### Adapter Boards
Some LiDAR/LDS models do not have built-in motor control and therefore require an additional board to operate:
- for Xiaomi Roborock 1st gen LDS02RR use [this board](https://github.com/makerspet/pcb/tree/main/lds02rr_adapter)
- for Neato XV11 use [this board](https://github.com/makerspet/pcb/tree/main/neato_delta_adapter)
- for 3irobotix Delta-2A, -2B, -2C PRO, -2D, -2G use [this board](https://github.com/makerspet/pcb/tree/main/neato_delta_adapter)

### ESP32 Modules
- When using the ESP32 WROVER board, it's worth using pins 32 and 33 instead of pins 16 and 17, which are used for QSPI
- When using the YD ESP32-S3, consider the default TX1, RX1 GPIO15 and GPIO16 pins and change the example code to `HardwareSerial LidarSerial(1);`

## Performance Notes
- currently Delta-2A, -2B, -2G exhibit mysterious fixed pattern noise
  - this noise does NOT show when using 3irobotix development kit, so it should be fix-able (somehow)
  - so far I haven't heard back from 3irobotix support.

## Release notes

## v0.5.8
- LDROBOT LD14P commands
  - start, stop motor
  - set, get scan frequency

## v0.5.7
- added YDLIDAR X4-PRO

## v0.5.6
- added YDLIDAR SCL

## v0.5.5
- compilation bugfix

## v0.5.4
- added CAMSENSE X1
- example sketch fix

## v0.5.3
- added Delta-2A 230400 baud
- added Delta-2B

## v0.5.3
- Camsense X1

## v0.5.2
- added LDROBOT LD14P

## v0.5.1
- bugfix `lds_all_models.h` include file

## v0.5.0
- added Delta-2A, Delta-2G

## v0.4.0
- added Neato XV11
- added RPLIDAR A1
- added YDLIDAR X3, X3 PRO
- report scan RPM for all sensor models

### v0.3.1
- added YDLIDAR X2/X2L
- measure RPM for YDLIDAR X4, YDLIDAR X2/X2L

### v0.3.0
- virtual class methods
- ESP32 crash workaround by moving init code from constructor out to init()

### v0.2.0
- example bugfix
- renamed classes

### v0.1.0
- initial release

## TODO
- add Xiaomi Roborock LDS01RR
- add LDROBOT LD20
- add LDROBIT LD19P, https://github.com/Myzhar/ldrobot-lidar-ros2
- add Hitachi-LG HLS-LFCD2
- add Dreame TBD
- reduce raw data volume
  - omit measurement quality since it usually does not get used

## Star History
[![Star History Chart](https://api.star-history.com/svg?repos=kaiaai/LDS&type=Date)](https://star-history.com/#kaiaai/LDS&Date)
