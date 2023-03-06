---
title: Powering on the Vehicle Compute Server
icon: codespaces
order: 100
---

!!!danger
Please heed the warnings here very carefully. If you are unsure as to how to perform any step, ask someone
who has done it before. Do not deviate from the steps here at risk of damaging the vehicle or hurting yourself.
!!!

## Wall Power

### Connecting to Wall

1. First, connect the vehicle server to the wall via the screw-in cable in the lab.

!!!danger Charger Port
The vehicle server is _NOT_ powered by the vehicle charger, only turn on the server while
the vehicle server is _directly_ plugged in.
!!!

2. Ensure that the red power delivery switch is switched to the
   `Wall` option.

### Initializing Server

1. Engage the `System` breaker located next to the metal power supply on the top
   of the server chassis by pushing the black tab into the breaker. You will then hear the watercooling pump spin up. Be sure to wait
   for the cooling pump to fully spin up before proceeding.

2. Engage the `Server` breaker located by the `System` breaker by pushing the black tab into the breaker. Wait until you hear a beep
   to continue.

3. Plug in a TV to the dev server via HDMI. Ensure that you plug the TV into one of the HDMI ports
   from the server's GPU, not the singular HDMI port near other I/O.

4. Flip the server switch located on the front face of the server.

!!!info Server Switch
The server power button is located underneath all of our networking cables. There's a convenient hole in the networking
cables where the power button is located.
!!!

5. Wait for the server to spin up. It will take a minute, and may require a restart. If you see a SuperMicro logo, and then
   an Ubuntu colored boot screen, the server has initialized correctly. _Be patient, it won't boot as fast as your desktop/laptop_

6. Login and start working! You can initialize the Docker in `~/docker/` with `./run.sh`.

### Turning Off

1. Shutdown the server in Ubuntu.

2. Hit the red button on the `Server` switch.

3. Hit the red button on the `System` switch.

4. Turn the red power delivery option to off.

5. Unscrew/unplug the screw-in cable for power delivery.

## Battery Power

### Preparing the Vehicle to Use Battery Power

1. Turn on the vehicle.

!!!danger
**You MUST turn on the vehicle before proceeding. If you do not, the server will drain the vehicle's batteries.**
!!!

2. Fill out pre-drive checklist. Paper copies of the pre-drive checklist are located in the vehicle, and
   an online form-based version of the checklist is coming soon.

!!!danger
**Do not drive the vehicle unless you have been approved by the ECE department to drive. Doing so will result in severe
consequences if discovered. Ensure you fill out the pre-drive checklist. Failing to check the vehicle before driving may result
in system failures, and you may be held liable if the checklist has not been filled out**
!!!

3. Ensure that the red power delivery switch is switched to the
   `Vehicle` option.

4. Ensure that the vehicle is disconnected from wall power.

### Initializing Server

1. Engage the `System` breaker located next to the metal power supply on the top
   of the server chassis. You will then hear the watercooling pump spin up. Be sure to wait
   for the cooling pump to fully spin up before proceeding.

2. Engage the `Server` breaker located by the `System` breaker. Wait until you hear a beep
   to continue.

3. Plug in a TV to the dev server via HDMI. Ensure that you plug the TV into one of the HDMI ports
   from the server's GPU, not the singular HDMI port near other I/O.

4. Flip the server switch located on the front face of the server.

!!!info Server Switch
The server power button is located underneath all of our networking cables. There's a convenient hole in the networking
cables where the power button is located.
!!!

5. Wait for the server to spin up. It will take a minute, and may require a restart. If you see a SuperMicro logo, and then
   an Ubuntu colored boot screen, the server has initialized correctly. _Be patient, it won't boot as fast as your desktop/laptop_

6. Once the login screen is displayed, disconnect the TV from the server. You can now shut the trunk of the vehicle.

7. Connect a laptop to the server via the Ethernet cable under the passenger seat in the front.

8. SSH into the server with command: `ssh autodrive@192.168.1.1` and login. You can now initialize the docker,
   and do whatever you need.

### Turning Off

!!!danger Turning the Vehicle Off
Do not turn off the vehicle before you have completely turned the server off.
!!!

1. Shutdown the server in Ubuntu.

2. Hit the red button on the `Server` switch.

3. Hit the red button on the `System` switch.

4. Turn the red power delivery option to `Off` from `Car`.

5. Turn off the vehicle.
