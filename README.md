# LDS/LiDAR Library for Arduino
Laser distance scan sensor (LDS/LIDAR) Arduino wrapper/controller for [kaia.ai](https://kaia.ai) platform.
Please read this [blog post](https://kaia.ai/blog/arduino-lidar-library/) for more details.

This library supports:
- YDLIDAR X4, X3, X3-PRO, X2/X2L models
- SLAMTEC RPLIDAR A1
- Neato XV11/Botvac
- Xiaomi Roborock Mi 1st gen LDS02RR
- 3drobotix Delta-2A, Delta-2G

Other models are in the works.

![lidar_sensors](https://github.com/kaiaai/LDS/assets/33589365/c38af37b-a9b1-44d1-b256-94d72e7562c6)

### Video: Neato XV11 runs on Arduino
<a href="http://www.youtube.com/watch?feature=player_embedded&v=kfk1Q0RSJpI" target="_blank">
 <img src="http://img.youtube.com/vi/kfk1Q0RSJpI/maxresdefault.jpg" alt="Neato XV11 laser distance scan sensor runs on Arduino" width="720" height="405" border="10" />
</a>

### Video: Xiaomi Mi 1st gen LDS02RR runs on Arduino
<a href="http://www.youtube.com/watch?feature=player_embedded&v=gaDnZ4Msw0E" target="_blank">
 <img src="http://img.youtube.com/vi/gaDnZ4Msw0E/maxresdefault.jpg" alt="LDS02RR laser distance scan sensor runs on Arduino" width="720" height="405" border="10" />
</a>

### Video: SLAMTEC RPLIDAR A1 runs on Arduino
<a href="http://www.youtube.com/watch?feature=player_embedded&v=f8IYjfiXsMk" target="_blank">
 <img src="http://img.youtube.com/vi/f8IYjfiXsMk/maxresdefault.jpg" alt="RPLIDAR A1 laser distance scan sensor runs on Arduino" width="720" height="405" border="10" />
</a>

Some LiDAR/LDS models do not have built-in motor control and therefore require an additional board to operate:
- for Xiaomi Roborock 1st gen LDS02RR use [this board](https://github.com/makerspet/pcb/tree/main/lds02rr_adapter)
- for Neato XV11 use [this board](https://github.com/makerspet/pcb/tree/main/neato_delta_adapter)
- for 3irobotix Delta-2A, -2B, -2C PRO, -2D, -2G use [this board](https://github.com/makerspet/pcb/tree/main/neato_delta_adapter)

## Release notes

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
- add LDROBOT LD14P, LD20
- add Hitachi-LG HLS-LFCD2
- add Dreame TBD
- reduce raw data volume
  - omit measurement quality since it usually does not get used