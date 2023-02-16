# Failsafe

Modules for breakdowns, accidents, and take over requests

    .
    ├── error_check.py : Check the heartbeat of all sensors and report errors
    ├── estop_check.py : Report the status of the serially connected e-stop button
    ├── mode_check.py : Check the final mode for the state of the mode
    └── tor_check.py : Reports on user-selected mode or mode change due to ToR
