# Droneport

## Goal
The main goal of the Droneport is to provide an autonomous system with which unmanned aerial vehicles can participate in industrial processes an it is formed by several modules, themost important ones are: housing, autonomous charging and communications. The first one protects the drone from the outdoor weather and is provided with mechanisms that will let it
fly freely. The second one lets the drone have a charged battery without the intervention of a person. Finally, the IoT based communications module lets the user operate the drone andthe station remotely.

## Implementation
The software is based on Python 3, using [MAVSDK](https://mavsdk.mavlink.io/develop/en/index.html) as platform to build connection from the main computer using a Debian based OS and a 915MHz antenna.Rest of communications are done via WebSockets following MQTT protocol, and some HTML, CSS and JavaScript.
