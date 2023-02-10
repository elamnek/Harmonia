# Harmonia Research Submarine
Harmonia research submarine onboard control software. This is the main control code for the research submarine. It collects data from sensors, controls the servos, main motor, peristaltic pump and pitch control actuator. Please refer to Wiki for more detailed information:

https://github.com/elamnek/Harmonia/wiki

https://user-images.githubusercontent.com/25494253/217131320-b1a3298b-334c-4569-9434-a887563058a7.mp4

Related projects are: 

HarmoniaRemote - windows desktop software to control operation of submarine (via RF transciever) and collect sensor data and forward to PostgreSQL database
https://github.com/elamnek/HarmoniaRemote


HarmoniaSlave - a secondary microprocessor that sits in the aft section of the sub and acts as a sensor slave to pass RPM and internal pressure sensor data to main microprocessor
https://github.com/elamnek/HarmoniaSlave

Note: the Harmonia submarine is part of a University of Adelaide research project being carried out between mid 2022 and mid 2023 - this repository will be continually updated over the next few months with the latest code as it is developed. Final release will be published in mid 2023.
