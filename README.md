# Blueshift

Repository for our what would have been our 2020 ASME entry, Blueshift. Although the vehicle was never completed the electronics were.

The system is heavy based off of TITAN given they are both video system for the riders. However it has some changes:
- Lighting hardware (and control) to meet road legality requirements
- Hardware was designed as an RPi HAT for a more compact system.
- Telemetry was operated by the microcontroller, not the RPi as on TITAN.
- Moved from a Python based overlay system to C based one to improve overlay update rate and prevent the overlay blacking out the video on occasion.

All systems were tested from home during the first set of lockdowns during COVID and were successful. So it is likely they would have worked just as well had the bike been built.
