Provided as a reference example with no support other than through this repository. 

This script provides an example of how to use the CAN bus interface on a Cisco IR1800 with an OBD2 port in a vehicle.  Currently, the script works with most vehicles that utilize 11-bit CAN IDs.

The script simply runs in a loop.  With environment variables (found in package_config.ini), you can define what gets read on each loop.  For instance, you could have a loop that runs every 10 seconds, and you read the GPS stream every time through the loop, speed every time, rpm every time, but maybe odometer every 6 times around the loop (doesn't change that often, so we can conserve bandwidth).  The app currently checks for some of the more interesting standard PIDs, like speed, rpm, odometer, fuel level, air intake temp, and coolant temp.  This could be modified to gather any of the other standard PIDs or custom locations (ie. seatbelt status; provided that you know the PID/CAN ID for a given vehicle).  At the end of each loop, the data is published to the MQTT broker of your choice in a JSON format.  The MQTT topic pattern that you define will be appended with the router serial number, as well, so you can differentiate between routers/vehicles on the subscribing side.

IMPORTANT NOTE: The CAN bus baud rate needs to be set properly in the IR1800's configuration.  Typically, this will be 500kbps for an OBD2 port.  The command to set this in the IR1800's config is: 
canbus baudrate 500000
