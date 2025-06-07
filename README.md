# Common Workspace Management System

## Project Overview
This project involves the design and development of an embedded surveillance and control system for a shared workspace or meeting room. The system monitors environmental conditions such as temperature, light levels, and occupancy, while also providing alert functionalities for emergencies. The system is built around the STM32F401VE microcontroller and integrates various sensors and actuators to ensure optimal workspace conditions.

## Features
1. **Temperature Monitoring and Control**  
   - Uses the LM35 temperature sensor to monitor ambient temperature.  
   - Displays real-time temperature on an LCD screen.  
   - Activates a DC motor fan when the temperature exceeds 25°C and deactivates it when it drops below 18°C.  
   - Implements a timer to pause the fan every 30 minutes for 10 minutes once activated.

2. **Occupancy Management**  
   - Detects presence using a push-button sensor.  
   - Limits occupancy to a maximum of 8 people.  
   - Uses green and red LEDs to indicate occupancy status (green for detection, red when full).  
   - Decrements the count when a person exits using a departure button.

3. **Lighting Control**  
   - Monitors ambient light levels using an LDR sensor.  
   - Automatically adjusts lighting by turning 2 lamps on/off based on the detected light levels for energy efficiency.

4. **Alert System**  
   - Includes an emergency push-button to trigger alerts.  
   - Activates a buzzer and displays a blinking "A" on a BCD display when pressed.  
   - Disables room access during an alert.

5. **Data Storage**  
   - Uses the internal EEPROM of the STM32F401VE to store temperature thresholds and the total number of alerts triggered.

6. **LCD Display**  
   - Shows a welcome message ("Bienvenue") at startup.  
   - Displays real-time temperature and occupancy status.

## Hardware Components
- Microcontroller: STM32F401VE (8MHz)  
- Sensors: LM35 (temperature), LDR (light), push-button (presence/departure/alert)  
- Actuators: DC motor fan, 2 lamps, buzzer, green/red LEDs  
- Displays: 16x2 LCD, 7-segment BCD display  

## Software Requirements
- Embedded C programming for STM32F401VE.  
- Proteus (ISIS) for simulation.  

## Project Structure
The project is divided into five parts, each focusing on specific functionalities:
1. **Part 1**: STM32F401VE architecture study, LCD setup, and initial system design.  
2. **Part 2**: ADC configuration, temperature monitoring, and ventilation control.  
3. **Part 3**: Timer integration for fan control, alert system, and lighting management.  
4. **Part 4**: EEPROM usage for storing thresholds and alert counts.  
5. **Part 5**: Final validation.  

## Repository Contents
- `Source code/`: Contains C source files for each part of the project.  
- `Proteus/`: Includes Proteus simulation files and circuit diagrams.  
