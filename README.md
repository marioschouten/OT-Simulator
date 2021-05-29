# OT-Simulator
## OpenTherm boiler simulator

### Hardware:
- Arduino Wemo D1 mini
- Ihor Melnyk's slave Terminal adapter for communication.
- 1-Wire thermometers for boiler water temperature and return temperature

### Software:
The software will simulate the function of a central heating system and answer the incoming messages from a thermostat. The software can be controlled with MQTT messages, the reporting is send on MQTT:

### The following is implemented: 
* The commands to update setpoints are set via MQTT topic [command] in the format as described below.
* The heater operational status is set via MQTT topic [status] with the format as described below.
* The various measurements of temperature, pressure, flow etc. are set via MTT topic [sensors] as described below for the purpose of testing the software or feedin external sensor readings.

**Messages to thermostat**
Topic | Description
------|------------
thermostat/ch_requested | CH requested
thermostat/modulation | Modulation requested by thermostat
thermostat/boilertemp | Boiler temperature
thermostat/returntemp | Return temperature

**COMMANDS to override defaults if required**
topic | default | Description
------|------------|--------
command/max_rel_modulation | 100 | max_rel_modulation
command/control_ch_setpoint |75 | control_ch_setpoint
command/max_ch_water_setpoint | 85 | max_ch_water_setpoint
command/dhw_setpoint | 0 | dhw_setpoint

**SENSORS value input for testing the software is no OpenTherm thermostat is availble**
topic | default value
------|------------
sensors/water_pressure_ch | 0 
sensors/outside_temperature | 0  
sensors/heater_flow_temperature | 0  
sensors/return_water_temperature | 0  
sensors/water_flow_dhw | 0 
sensors/dhw_temperature | 0

**STATUS messages**
Topic | Description
------|------------
status/fault | 0 | Fault indication 
status/ch_mode | 0 | CH Mode
status/flame | 0 | Flame status
_not supported_ | _0_ | _DHW Mode_
_not supported_ | _0_ | _Cooling status_
_not supported_ | _0_ | _CH2 Mode_
_not supported_ | _0_ | _Diagnostics_
_not supported_ | _0_ | _Reserved_

