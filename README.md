# BLAST-TEAM7

## Project Overview

The BLAST-TEAM7 project focuses on developing and testing a system of Unmanned Surface Vehicles (USVs) that communicate using LoRa (Long Range) radio modules and GPS for coordinated search and rescue operations. The system includes Arduino sketches for the USVs and the central station, as well as a Python GUI for monitoring and visualization.

[USV_SOS-USV.pdf](./USV_SOS-USV.pdf) - USV Board Schematic. 

[USV_SOS-SOS_Beacon.pdf](./USV_SOS-SOS_Beacon.pdf) - SOS Beacon Schematic

## Directory Structure

- **Central_Arduino/**
  - `Central_Arduino.ino` - Arduino sketch acting as a central station that listens for data broadcasted by the USVs.
  
- **SOS_Beacon/**
  - `SOS_Beacon.ino` - Arduino sketch for the SOS beacon that broadcasts an emergency signal.

- **TESTS/**
  - **GPS PA1010D I2C/**
    - **GPS_I2C_EchoTest/**
      - `GPS_I2C_EchoTest.ino` - Test sketch for echoing GPS data over I2C communication.
    - **GPS_I2C_Parsing/**
      - `GPS_I2C_Parsing.ino` - Test sketch for parsing GPS data received via I2C.
  - **RFM95W LORA TESTS/**
    - **rf95_client/**
      - `rf95_client.ino` - Arduino sketch for a client node using the RFM95W LoRa module.
    - **rf95_server/**
      - `rf95_server.ino` - Arduino sketch for a server node using the RFM95W LoRa module.
  - **RFM96W LORA TESTS/**
    - **rf96_client/**
      - `rf96_client.ino` - Arduino sketch for a client node using the RFM96W LoRa module.
    - **rf96_server/**
      - `rf96_server.ino` - Arduino sketch for a server node using the RFM96W LoRa module.

- **USV/**
  - `USV.ino` - Arduino sketch for controlling an Unmanned Surface Vehicle (USV) that communicates with other USVs and a central station using LoRa modules and GPS.

- **USV1/**
  - `USV1.ino`

- **USV2/**
  - `USV2.ino`

- **USV3/**
  - `USV3.ino`

- **USV_Dashboards.ipynb** - A Jupyter notebook for generating dashboards related to USV data and simulations.

- **usv_gui.py** - Python script for a graphical user interface (GUI) to display data received from the central station.

- **usv_simulator.py** - Python script for simulating USV operations.

## Detailed Component Descriptions

### `USV.ino`

- **Purpose**: This Arduino sketch is designed for an Unmanned Surface Vehicle (USV) that communicates with other USVs using LoRa (Long Range) radio modules (RFM95 and RFM96) and also uses GPS for location data.
- **Functionality**:
    - Sets up two LoRa modules, one for communication with other USVs and one for receiving SOS signals.
    - Maintains the USV's status, including grid position, latitude, longitude, signal strength, and ID.
    - Broadcasts its status to other USVs.
    - Listens for incoming SOS signals and triggers convergence on the signal.
    - Contains functions to estimate positions and triangulate based on signal strength.

### `Central_Arduino.ino`

- **Purpose**: This sketch acts as a central station that listens for data broadcasted by the USVs.
- **Functionality**:
    - Initializes a LoRa radio to receive messages.
    - Listens for incoming USV status data and prints it via serial.
    - Uses a similar structure (`USVStatus`) to the USVs, indicating it expects data in the same format.

### `usv_gui.py`

- **Purpose**: A Python script with a GUI to display data received from the central station.
- **Functionality**:
    - Uses a serial connection to read data transmitted by the central Arduino.
    - Displays data in a text area and plots USV positions and estimated SOS locations on a map.
    - Processes data lines, expecting to parse messages that start with `DATA` or `ESTIMATED`.
    - Visualizes USV positions and the estimated SOS location, updating dynamically as new data is received.

## Integration and Interaction

### Communication Flow

- **SOS Beacon**: 
  - Broadcasts an emergency signal at 433 MHz using the RFM96W LoRa module.
  
- **USVs**:
  - Detect the SOS signal using the RFM96W module, gather GPS data and signal strength, and communicate with each other at 915 MHz using the RFM95W module.
  - After determining the estimated location of the SOS source, transmit this information to the Central System using the RFM95W module.

- **Central Arduino**:
  - Acts as the recipient of the USV broadcasts. It decodes the `USVStatus` structure and outputs the data to the serial port.
  - The `usv_gui.py` script reads from this serial port, processes the data, and updates the GUI to show USV positions and signal strengths.

## Dependencies

### Hardware

- **USVs** require specific LoRa modules (RFM95W and RFM96W) and a GPS module.

### Software

- **Arduino IDE** for working with `.ino` files.
- **Python 3.x** and necessary libraries for running the Python scripts and Jupyter Notebook.

### Serial Connection

- The GUI expects a serial connection (e.g., `COM4`) to the central Arduino, which needs to be adjusted based on the actual configuration.

## Installation and Setup

1. **Clone the Repository**:
   ```bash
   git clone https://gitlab.jhuapl.edu/travis.latchman/blast-team7.git
