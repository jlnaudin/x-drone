x-Drone 
=======
Experimental researches and developpement of autopilots for aerial plateforms on Arduino boards:
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/FX61Phantom/FX61Phantom02.jpg "The FX-61 Phantom APM v2.6 setup")
* ArduPilot Mega (APM v1.5, APM v2.x) from 3D Robotics
* ArduFlyer v2.5.2 from RC Timer
* APM v2.6 ArduPilot Mega from HobbyGaga

**All these informations are published free in Open Source under [GNU licence] (https://github.com/jlnaudin/x-drone/blob/master/LICENSE) for a non-commercial and a private use only**


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

[**FULL TUTORIALS** about the ArduPlane v2.73-xp1 installation,tuning parameters and HIL simulations] (https://github.com/jlnaudin/x-drone/wiki)

[**FULL VIDEOS** about the ArduPlane v2.73-xp1 testing and real flights missions] (https://github.com/jlnaudin/x-drone/wiki/Videos-of-the-Calmato-missions-piloted-with-ArduPlane)

#Latest News
* [**x drone: FX 61 Phantom flying wing tested in a virtual wind tunnel**]
(https://github.com/jlnaudin/x-drone/wiki/x-drone:-FX-61-Phantom-flying-wing-tested-in-a-virtual-wind-tunnel)
* [**x drone: FX 61 Phantom flying wing in autonomous thermalling with the ThermoPilot v7.2**]
(https://github.com/jlnaudin/x-drone/wiki/x-drone:-FX-61-Phantom-flying-wing-in-autonomous-thermalling-with-the-ThermoPilot-v7.2)
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/FX61Phantom/fx61thermalling01.jpg "The FX-61 Phantom")

* [**X PLANE TUTORIAL: X Plane v10 with ArduPlane v2.73 xp1 in HIL mode simulation**]
(https://github.com/jlnaudin/x-drone/wiki/X-PLANE-TUTORIAL:-X-Plane-v10-with-ArduPlane-v2.73-xp1-in-HIL-mode-simulation)
* [**x drone: MaxiSwift, mission 35 comparison of FPL path of Real flight Vs HIL simulation**] (https://github.com/jlnaudin/x-drone/wiki/x-drone:-MaxiSwift,-mission-35---comparison-of-FPL-path-of-Real-flight-Vs-HIL-simulation)
* [**x drone: MaxiSwift, mission 28 Windy and Gusty mission test flight**] (https://github.com/jlnaudin/x-drone/wiki/x-drone:-MaxiSwift,-mission-28---Windy-and-Gusty-mission-test-flight)
 
***

* [**MaxiSwift flying wing wiki**] (https://github.com/jlnaudin/x-drone/wiki/x-drone:-MaxiSwift,-mission-8---Low-altitude-profile-mission)
* [**MaxiSwift for X-plane v9.70 and HIL simulations with the APM**] (https://github.com/jlnaudin/x-drone/wiki/X-PLANE-TUTORIAL:-MaxiSwift-installation-with-X-Plane-v9.70-for-HIL-simulations)

#VIDEOS

[![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/FX61Phantom/FX61thermopilot72.jpg)](http://player.vimeo.com/video/87475677)

[![alt text](https://raw.github.com/jlnaudin/x-drone/master/MaxiSwift/MaxiSwiftMission8tittle.jpg)](http://player.vimeo.com/video/81333614)

[![alt text](https://raw.github.com/jlnaudin/x-drone/master/mission_logs/Calmato_Mission18.jpg "Mission 18 video")]
(http://player.vimeo.com/video/80605879)

[![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/CalmatoFlight.jpg)] (http://player.vimeo.com/video/79610014)

[![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/APN273xp1FullTest.jpg)] (http://player.vimeo.com/video/80052308)

#PHOTOS
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/FX61Phantom/fx61thermalling19.jpg "The FX-61 Phantom")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_hil_mode2.jpg "The Calmato for X-Plane")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_real_mode1.jpg "The real Calmato tested in flight")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_real_mode2.jpg "The real Calmato tested in flight")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmatodrone.jpg "The Calmato Alpha 40 with the Ground Station")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/CalmatoMSwing.jpg "The Calmato Alpha 40 and the MaxiSwift tested with ArduPlane")


![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_hil_mode1.jpg "The Calmato for X-Plane")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_real_mode3.jpg "The real Calmato tested in flight")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_real_mode4.jpg "The real Calmato tested in flight")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/calmato_real_mode5.jpg "The real Calmato tested in flight")
**The FPL recorded in flight during the mission 16**
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/CalmatoMission16FPL.jpg "The FPL recorded in flight during the mission 16")
**The flight log of the mission 16**
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/CalmatoMission16.jpg "The flight log of the mission 16")
![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/APM26miniOSD.jpg "APM v2.6 setup")

![alt text](https://raw.github.com/jlnaudin/x-drone/master/images/ArduFlyerSetup.jpg "ArduFlyer v2.5.2 AP setup")

***
Visit the x-VTOLdrone area: https://github.com/jlnaudin/x-VTOLdrone

More infos at: http://diydrones.com/profile/JeanLouisNaudin

Flights videos at: http://vimeo.com/user5037815/videos
