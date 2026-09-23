# Robot POS

The POS is plain HTML, CSS, and browser JavaScript. The Python controller serves
these files from the same process that runs the delivery FSM; Node.js and a
frontend build step are not required.

## Running

Start the robot controller from the repository root:

    python3 src/main.py

The page is served only on 127.0.0.1:8765. It is available at
http://127.0.0.1:8765/ after the controller starts. The POS does not issue
motion commands itself; it submits a mission to the Python FSM.

For a mission, choose a destination for each used shelf and confirm that food
has been placed on every selected shelf. The robot visits shelf 1's table
before shelf 2's table. At each destination, confirm pickup on the touchscreen.
The physical override button remains available as an alternate pickup
confirmation. A browser reload reconnects to the current controller state.

## Ubuntu Desktop kiosk startup

On a Pi configured for automatic desktop login, use the user-level systemd
unit. It starts with the kiosk user's desktop session and does not require a
root-owned unit or lingering. Replace __POS_PROJECT_DIR__ with the absolute
repository path in deploy/food-delivery-pos.user.service.example, then run:

    mkdir -p ~/.config/systemd/user ~/.config/autostart
    sed 's|__POS_PROJECT_DIR__|/absolute/path/to/EmbeddedProj|g' deploy/food-delivery-pos.user.service.example > ~/.config/systemd/user/food-delivery-pos.service
    sed 's|__POS_PROJECT_DIR__|/absolute/path/to/EmbeddedProj|g' deploy/food-delivery-pos.desktop.example > ~/.config/autostart/food-delivery-pos.desktop
    systemctl --user daemon-reload
    systemctl --user enable --now food-delivery-pos.service

The service launches deploy/run_pos_controller.sh, which sources the ROS 2
Jazzy setup and the default workspace at ~/ros2_ws/install when present. Set
ROS_SETUP or ROS_WS_SETUP in the unit if those setup files are elsewhere. The
desktop entry starts deploy/start_pos_kiosk.sh after a short delay. It guards
against duplicate launches, logs to /tmp/pos_kiosk_autostart.log, waits for the
local health endpoint, and restarts the browser if it exits. The browser
launcher checks for chromium, chromium-browser, then Firefox. Chromium runs in
kiosk mode. Firefox uses a separate POS profile and opens a new window; on the
Ubuntu Pi's Snap Firefox, kiosk mode stayed alive but did not navigate to the
local POS URL, while a normal window loaded it successfully. If the executable
has a different name, set POS_BROWSER near the top of deploy/pos_kiosk.sh.

Ubuntu must be configured to automatically log in to the kiosk account so its
user service and desktop autostart run after boot. The account needs access to
the robot serial device (normally membership in the dialout group). A system
service template is also provided for setups that need the controller before
desktop login; that alternative requires sudo and automatic desktop login for
the browser. The POS port defaults to 8765.
