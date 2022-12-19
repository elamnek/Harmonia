# Harmonia
Harmonia research submarine onboard control software. This is the main control code for the research submarine. It collects data from sensors, controls the servos, main motor, peristaltic pump and pitch control actuator. Please refer to Wiki for more detailed information:

https://github.com/elamnek/Harmonia/wiki

Related projects are: 

HarmoniaRemote - windows desktop software to control operation of submarine (via RF transciever) and collect sensor data and forward to PostgreSQL database
https://github.com/elamnek/HarmoniaRemote


HarmoniaSlave - a secondary microprocessor that sits in the aft section of the sub and acts as a sensor slave to pass RPM and internal pressure sensor data to main microprocessor
https://github.com/elamnek/HarmoniaSlave

