# Smart Core Thermostat

This is the firmware code for my personal project to manage my electric water heater with a Particle (ex Spark) Core.

Dependencies
---
OneWire: https://github.com/particle-iot/OneWireLibrary
SparkIntervalTimer: https://github.com/pkourany/SparkIntervalTimer
Spark-RestClient: https://github.com/llad/spark-restclient


Potentiometer calibration and EEPROM
---
We need a factor to scale the potentiometer readings to 0-100°C.
A dynamic way to do this can be get PMAX and PMIN values:
- in setup(): check for already saved PMIN,PMAX data in EEPROM. If not, start
a 5 seconds calibration procedure. This procedure is getting max and min READINGS
from potentiometer. When finished, save data in EEPROM
- in normalize_temperature_reading(): use PMIN and PMAX loaded in setup() in a map
call
It can be useful to add something (perhaps a button?) to manually trigger the
 calibration.

My Watchdog implementation
---
A timer is calling each 10 seconds a callback. This callback is resetting the Core.
Inside the main loop function we reset the timer each 2.5 seconds
If the Core is stuck for more than 10 seconds, the callback will be called resetting the Core.

Remote Logging
---
Client side (.ino file): I know I can do much better.
Server side: Seriously need improvements. It's the node.js UDP server sample code
