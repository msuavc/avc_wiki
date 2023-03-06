---
title: Data Collection
icon: database
order: 99
---

!!!danger Approved Drivers
_Drivers_ of the vehicle need to be preapproved by the ECE department prior to operating the vehicle.
Do not seek out approval yourself, ask a lead.

_Passengers_ do not need to be preapproved. As a passenger, make sure your driver is preapproved.
!!!

1. Follow the steps [here](<~/vehicle/power-delivery#Battery Power>) to initialize the server for battery power.

2. Once you have connected to the server via SSH, initialize the docker:

   ```bash
   cd docker/
   ./run.sh
   ```

   You should then see a modified prompt for the docker: `[DOCKER]`

3. Start a `tmux` session inside the docker container. The [default keybinds](https://man7.org/linux/man-pages/man1/tmux.1.html#:~:text=DEFAULT%20KEY%20BINDINGS%20top,in%20the%20current%20window%20forwards.)
   are configured.
   For quick reference:

   ```Keybinds
   CTRL-B + " : Split window horizontally
   CTRL-B + % : Split window vertical
   CTRL-B + C : Create a new window
   CTRL-B + N : Move to next window
   CTRL-B + index : Move to window at index, ex: CTRL-B + 0 will move to the first created window
   ```

4. Use roslaunch to launch the system for data collection in one window/pane:

   ```bash
   roslaunch bolt_launch data_collection.launch
   ```

5. Using the `rostopic` commandline tool from a separate tmux window/pane:
   1. List all topics to make sure that they are all being published
   2. Ensure that the most important topics are publishing:
      - Lidar
      - Radar
      - Cameras
      - GNSS
      - IMU

!!!info Topic Analyzer
A TUI is in development to simplify validating topic uptime.
!!!

6. If you have ensured the integrity of topis, move the car outside.

7. Start up the data collection script in a separate tmux window/pane:

   ```bash
    ./tools/data_collection.sh bag_name
   ```

   - `bag_name` should be descriptive relative to whats happening with the weather. Examples:
     1. `campus_snow` for snowy weather
     2. `campus_rain` for rainy weather
     3. `campus_sunny` for sunny weather

8. Drive the planned route.

9. Park the vehicle in the marked area, and **DO NOT TURN OFF THE CAR**.

!!!danger Turning off the Car
If you turn off the vehicle prior to turning off the dev server, the batteries will drain.
!!!

10. Plug the vehicle into lab ethernet, and rsync the bags with our campus storage:
    ```bash
    ssh -L 9922:davinci.egr.msu.edu:22 {username}@compute01.egr.msu.edu
    rsync --progress -av -e "ssh -p 9922" /mnt/internal-datasets/{bag_name}*.bag \
      {username}@localhost:/egr/research-canvas/raw_datasets/campus/
    ```
