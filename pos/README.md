# Robot POS

The POS is plain HTML, CSS, and browser JavaScript. The Python controller serves
these files from the same process that runs the delivery FSM; Node.js and a
frontend build step are not required.

## Running the delivery system

On the Raspberry Pi, start the complete delivery stack from the repository root:

    bash start_robot.sh

The script starts LiDAR, the ROS bridge, SLAM Toolbox, the POS controller, and
the POS kiosk in one persistent tmux session. The kiosk opens on the Pi's local
desktop display. The desktop session must be logged in as the same user that
runs the script. See [the setup guide](../docs/setup.md) for SSH operation,
display settings, and readiness checks.

The POS server listens only on 127.0.0.1:8765. To use it from another computer,
forward the port over SSH:

    ssh -N -L 8765:127.0.0.1:8765 <user>@<IP-of-Pi>

Then open http://127.0.0.1:8765/ on that computer. The POS submits missions to
the Python FSM; it does not send motor commands directly.

For controller-only debugging, `bash deployPOS/run_pos_controller.sh` starts
the POS and FSM but does not start the ROS bridge, LiDAR, or kiosk.

For a mission, choose a destination for each used shelf and confirm that food
has been placed on every selected shelf. The robot visits shelf 1's table
before shelf 2's table. At each destination, confirm pickup on the touchscreen
or press `#` on the physical keypad. The setup draft is stored by the Python
controller, so touchscreen and keypad actions update the same data and a browser
reload restores the current selections.

The 4x4 keypad is connected to the motion Arduino through a PCF8574 on A4/A5.
In setup mode use `A`/`B` for shelf 1/2, `1`/`2` for the table, `C` to confirm
food loading, `D` to clear the active shelf, `#` to start, and `*` to clear all.
While waiting at a table `#` confirms pickup; in `ERROR`, `*` requests a mission
reset. Keys are ignored while the robot is moving.

## Ubuntu Desktop kiosk startup

For automatic startup at boot, configure a desktop auto-login session and a
service that starts `start_robot.sh` as the same user. The provided systemd
examples are controller-only and their `ExecStart` still points to the old
`deploy/` directory instead of `deployPOS/`; they do not start the full
ROS/LiDAR stack and are not ready to install as-is. For the supported manual
startup and SSH workflow, use [the setup guide](../docs/setup.md).
