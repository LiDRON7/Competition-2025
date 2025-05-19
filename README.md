# Competition-2025
## How do I make the drone fly?

The main code used to fly the drone in Competition 2025 is located in ./Pixhawk. There are some test scripts that can be run to verify that everything is working. The script that has "everything" in place—camera detection, waypoints, and GPS data collection—is ./Pixhawk/waypoints_with_camera_showcase1.py.

Sometimes the GPS doesn't work when launching a script, so it's important to first run:

    $ mavproxy.py --master=/dev/ttyAMA0

Then, inside the MAVProxy interface, run:

    $ status GLOBAL_POSITION_INT

Running this command ensures the GPS is initialized and able to return coordinates when the Python scripts are executed.
