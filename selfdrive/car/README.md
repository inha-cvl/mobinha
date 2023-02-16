# Car

Car specific code to read states and control actuators

    .
    ├── fingerprint.json : for Parameter setting
    ├── can_bridge.py : CCAN->SCC, SCC->CCAN bridge
    ├── carstate.py : Openpilot code that receives information from CAN and stores it as carstate
    ├── dbc/niro
        ├── cankey.py : For successful connection between SCC and CCAN
        └── hyundai_can.dbc : Hyundai NIRO dbc file
    └── blinker : Blinker controller ( aduino )
