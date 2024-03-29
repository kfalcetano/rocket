## UMN Ski-U-Launch HPR Competition Code
Included are the Arduino codes sketches that were developed for the avionics telemetry system as part of my team's entry in the Minnesota Space Grant Consortium's **2018-2019 Midwest High-Power Rocketry Competition**. Both the ground station transciever unit and the payload suite files are included, and the naming as "test builds" is merely an artifact of the poor version control. Our "version control" was done in Google Drive, but I wanted it in Github now that I understand git better. This is no longer being worked on and is hosted here for ease of access documentation after the fact.

### Mission
The goal was to log altimiter, accelerometer, and gyro data to an SD card logger, and transmit over LoRa packet radio after main chute deployment. Following transmission of logs, the GPS module's sentences start being parsed for coordinates and those are then sent every few seconds over LoRa.

### Rough Functional Outline
1. Start up and verify component are go using LoRa packets as feedback to ground station
1. Wait for GPS fix so we can take advantage of hot start later
    * Able to skip using command from ground station if fix not achieved
1. Wait for axial acceleration spike for launch detection
    * Can initiate logging manually for more complete data
1. Begin logging
1. Wait for descending rate and altitude of less than 600 ft
1. Stop logging and transmit logs over LoRa
1. Transmit log to completion
1. Begin transmitting GPS coords