# LDS/LiDAR Library for Arduino
Laser distance scan sensor (LDS/LIDAR) Arduino wrapper/controller for [kaia.ai](https://kaia.ai) platform. The library supports
- YDLIDAR X4
- YDLIDAR X2
- Xiaomi Mi LDS02RR
- TODO Neato XV/Botvac

Note: LDS02RR, Neato LDS require an additional motor control board to operate.
You can find the board schematics at [Makerspet repo](https://github.com/makerspet/makerspet_snoopy/) under `kicad` or purchase it at [makerspet.com](https://makerspet.com) when it becomes available.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=gaDnZ4Msw0E" target="_blank">
 <img src="http://img.youtube.com/vi/gaDnZ4Msw0E/maxresdefault.jpg" alt="LDS02RR laser distance scan sensor operating" width="720" height="405" border="10" />
</a>

## Release notes

### v0.3.2
- added YDLIDAR X3 PRO

### v0.3.1
- added YDLIDAR X2
- measure RPM for YDLIDAR X4, YDLIDAR X2

### v0.3.0
- virtual class methods
- ESP32 crash workaround by moving init code from constructor out to init()

### v0.2.0
- example bugfix
- renamed classes

### v0.1.0
- initial release