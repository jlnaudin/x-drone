x-Drone 
=======
Experimental researches and developpement of autopilots for aerial plateforms on Arduino boards:
* ArduPilot Mega (APM v1.5, APM v2.x) from 3D Robotics
* ArduFlyer v2.5.x from RC Timer

FULL TUTORIAL for testing yourself the HIL simulation on X-Plane v9.70 at:
* https://github.com/jlnaudin/x-drone/wiki/X-PLANE-TUTORIAL:-X-Plane-v9.70-with-ArduPlane-v2.73-xp1-in-HIL-mode-simulation

FULL INSTALLATION details of the Real Calmato Alpha 40 EP at:
* https://github.com/jlnaudin/x-drone/wiki/The-Calmato-Alpha-40-EP-setup-with-the-APM-ArduFlyer-IMU-board

Programming the GPS Crius CN06-V2 (Ublox NEO-6M):
* https://github.com/jlnaudin/x-drone/wiki/How-to-setup-the-GPS-Ublox-NEO-6M-(Crius)-CN-06-V2

![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/CalmatoFlight.jpg) (http://player.vimeo.com/video/79610014)
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/CalmatoDroneFlight.jpg) (http://player.vimeo.com/video/79410452)
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/APN273xp1FullTest.jpg) (http://player.vimeo.com/video/80052308)
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_hil_mode2.jpg "The Calmato for X-Plane")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_real_mode1.jpg "The real Calmato tested in flight")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_real_mode2.jpg "The real Calmato tested in flight")
![alt text](https://raw.github.com/jlnaudin/x-drone/d696cd643404489d5c261b987d7cffb1705ebdff/images/calmatodrone.jpg "The Calmato Alpha 40 with the Ground Station")
#Features
* Arduino Auto-Pilot and full IMU 3D stabilizer (9DOF, GPS, 3D magnetometer, airspeed sensor...)
* Ground Station (the Mission Planner from Michael Oborne) with full Mavlink telemetry
* Enhanced version of the ArduPlane v2.73 firmware (called Arduplane_273-xp1)
* - Automatic recording of WP in flight with the CH7 switch in STABILIZE mode (CH7_OPTION	= CH7_SAVE_WP)
* - Erasing and the FPL option with the CH7 switch in MANUAL mode
* - RTL can be activated in flight with the CH7 switch in AUTO mode
* - Auto-Take off and Auto-Landing improved and tested in HIL mode on X-Plane v9.70
* - Adaptative Bank Turn option if AUTO_WP_RADIUS is ENABLED
* - Closed loop FPL option, when the CLOSED_LOOP_NAV is ENABLED
* - fully compatible and tested with X-Plane v9.70 in HIL (Hardware In the Loop) mode
* HIL mode test flights on X-Plane with the the Calmato Alpha virtual model
* Real test flights with a true Calmato Alpha 40 (Kyosho) electric powered RC plane

![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_hil_mode1.jpg "The Calmato for X-Plane")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_real_mode3.jpg "The real Calmato tested in flight")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_real_mode4.jpg "The real Calmato tested in flight")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_real_mode5.jpg "The real Calmato tested in flight")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/True_Nav.jpg "Real test flight")
![alt text](https://raw.github.com/jlnaudin/x-drone/e70b80f9e0fd244902bcd2584a326f90f9756420/images/ArduFlyerSetup.jpg "ArduFlyer v2.5.2 AP setup")
More infos at: http://diydrones.com/profile/JeanLouisNaudin

Flights videos at: http://vimeo.com/user5037815/videos
