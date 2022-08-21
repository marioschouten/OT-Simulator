# OT-Simulator
## OpenTherm E-CV simulator

### Hardware:
- Arduino Wemo D1 mini
- Ihor Melnyk's slave Terminal adapter for communication.
- 1-Wire thermometers for boiler water temperature and return temperature

### Software:
The software will simulate the function of a central heating system and answer the incoming messages from a thermostat. The software can be controlled with MQTT messages, the protocol message reporting will be send via MQTT to /ecv/theremostat/rwadata/rx and ecv/thermostat/rawdata/tx

### Our implementation
Our implementation is a thermostat connected to a heat pump, the OpenTerm output of the heat pump is connected to the OT-Simulator. The OT-Simulator will send by MQTT a message to OpenHAB with the modulation and CH requested and OpenHAB will then switch the 3 coils of a E-CV in seven steps. The original E-CV is a 9kW on/off type modified using SSR to switch the three coils in different configurations to create 7 power levels.
### The following is implemented: 
* The commands to update setpoints are set via MQTT topic [ecv/command] in the format as described below.
* The heater operational status is set via MQTT topic [ecv/status] with the format as described below.
* The various measurements of temperature, pressure, flow etc. are set via MTT topic [ecv/sensors] or 1-wire sensors.


**Messages from thermostat to OpenHAB**
In our setup this is the decoded OpenTerm protocol comming from the heat pump with live temperature reading added. 

Topic | Description
------|------------
ecv/thermostat/ch_requested | CH requested
ecv/thermostat/modulation | Modulation requested by thermostat
ecv/thermostat/boilertemp | Boiler temperature 
tecv/hermostat/returntemp | Return temperature 


**COMMANDS to override defaults**
topic | default | Description
------|------------|--------
ecv/command/max_rel_modulation | 100 | max_rel_modulation
ecv/command/control_ch_setpoint |75 | control_ch_setpoint
ecv/command/max_ch_water_setpoint | 85 | max_ch_water_setpoint
ecv/command/dhw_setpoint | 0 | dhw_setpoint


**SENSORS value input**
topic | default value | notes
------|------------|------
ecv/sensors/water_pressure_ch | 0 | 
ecv/sensors/outside_temperature | 0 | 
ecv/sensors/heater_flow_temperature | 0 | 1-Wire sensor or MQTT
ecv/sensors/return_water_temperature | 0 | 1-Wire sensor or MQTT
ecv/sensors/water_flow_dhw | 0 | 
ecv/sensors/dhw_temperature | 0 | 


**STATUS messages**
Upon a ecv/thermostat/ch_requested changing from 0 to 1 our OpenHAB system with set the corresponding power level based on the ecv/thermostat/modulation and send back both a ecv/status/ch_mode = 1 and ecv/status/flame = 1 to let the heat pump know the E-CV heater is active.

Topic | Default | Description
------|------------|---------
ecv/status/fault | 0 | Fault indication 
ecv/status/ch_mode | 0 | CH Mode
ecv/status/flame | 0 | Flame status
_not supported_ | _0_ | _DHW Mode_
_not supported_ | _0_ | _Cooling status_
_not supported_ | _0_ | _CH2 Mode_
_not supported_ | _0_ | _Diagnostics_
_not supported_ | _0_ | _Reserved_

