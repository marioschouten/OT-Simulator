# OT-Simulator
##OpenTherm boiler simulator

###Hardware:
- Arduino Wemo D1 mini
- Ihor Melnyk's slave Terminal adapter for communication.
- 1-Wire thermometers for boiler water temperature and return temperature

###Software:
The software will simulate the function of a central heating system and answer the incoming messages from a thermostat. The software can be controlled with MQTT messages, the reporting is send on MQTT:


|#|topic|default|Description|
|1|command/max_rel_modulation |100 |max_rel_modulation |
|1|command/control_ch_setpoint |75 |control_ch_setpoint |
|1|command/max_ch_water_setpoint |85 |max_ch_water_setpoint |
|1|command/dhw_setpoint |0 |dhw_setpoint  |



THe 
Implemented functionality is as follows:
 - The commands to update setpoints are set via MQTT topic [command] in the format as described under HEATER SETTINGS

 - The heater operational status is set via MQTT topic [status] with the format as described under HEATER SETTINGS
 
 - The various measurements of temperature, pressure, flow etc. are set via MTT topic [sensors] as described under HEATER SETTINGS

 - The modulation level requested by the Thermostat is publish via MQTT topic [thermostat/modulation]
 - The CH requested by the Thermostat is publish via MQTT topic [thermostat/ch_requested]
 
 
 //HEATER STATUS SETTINGS - Default can be adjusted with MQTT message
const char* fault_indication = "0";         // Default = 0, updated with MQTT topic [status/fault]
const char* CH_mode = "0";                  // Default = 0, updated with MQTT topic [status/ch_mode]
const char* flame_status = "0";             // Default = 0, updated with MQTT topic [status/flame]
const char* DHW_mode = "0";                 // Default = 0, if DHW is present change to 1
const char* cooling_status = "0";           // Default = 0, no support in this software version for cooling status
const char* CH2_mode = "0";                 // Default = 0, no support in this software version for CH2 mode
const char* diagnostic = "0";               // Default = 0, no support in this software version for diagnostics
const char* msg_0_bit_7 = "0";              // Reserved



//HEATER SENSORS SETTINGS - Default can be adjusted with MQTT message
double water_pressure_ch = 0;               // Default =  0, updated with MQTT topic [sensors/water_pressure_ch]
double outside_temperature = 0;             // Default =  0, updated with MQTT topic [sensors/outside_temperature]
double heater_flow_temperature = 0;         // Default =  0, updated with MQTT topic [sensors/heater_flow_temperature]
double return_water_temperature = 0;        // Default =  0, updated with MQTT topic [sensors/return_water_temperature]
double water_flow_dhw = 0;                  // Default =  0, updated with MQTT topic [sensors/water_flow_dhw]
double dhw_temperature = 0;                 // Default =  0, updated with MQTT topic [sensors/dhw_temperature]

//DEBUG MESSAGE SETTING
const char* serial_monitor = "1";           // Default = 0, if set to 1 the OpenTherm traffic will be shown on the serial monitor
const char* serial_mqtt    = "1";           // Default = 0, if set to 1 all MQTT related debug messages are shown on the serial terminal
const char* serial_range   = "0";           // Default = 0, if set to 1 all range check debug messages are shown on the serial terminal
const char* serial_update  = "1";           // Default = 0, is set to 1 all value updates are shown on the serial terminal
const char* serial_convert = "0";           // Default = 0, if set to 1 all value to hex conversion debug messages are shown on the serial terminal
const char* serial_onewire = "1";           // Default = 0, if set to 1 the system will print a list of device addresses to the terminal
const char* serial_debug   = "1";           // Default = 0, if set to 1 debug messages are shown on the serial monitor
