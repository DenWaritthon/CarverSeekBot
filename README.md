# Controllino

This code is referenced from an CarverCAB older version. Remove references to `touchless_switch` and `Alcohol`.

# Change log
`update 2-06-2568`
* **Disable LED Command ALL** because it is no longer in use
* **Change Emer to realworld** Reversed the behavior of the 'Emer' switch: true now represents false, and false represents true
* **FIX issue bug** Resolved Rapid Callback Execution: A bug causing the Controllino to perform actions too frequently due to a misaligned coil callback has been fixed. This ensures more precise and controlled operation.

# Controllino Communication
The Controllino currently communicates using the Modbus/TCP protocol. This means it utilizes Modbus communication over a standard Ethernet (LAN) connection. The Controllino's IP address for this communication is fixed at 192.168.1.191.

# Controllino Coil Mapping Reference
## Sensor Readings 

| Coil Number | Function | Description |
|-------------|----------|-------------|
| 0 | emer | Emergency |
| 1 | limit_b_l | Back Left Limit |
| 2 | limit_b_r | Back Right Limit |
| 3 | limit_f_l | Front Left Limit |
| 4 | limit_f_r | Front Right Limit |


## Command Inputs

| Coil Number | Function     | Description                              | DISABLE   |
|-------------|--------------|------------------------------------------|-----------|
| 20          | start_motor  | Initiate the motor start sequence        |               |
| 21          | stop_motor   | Initiate the motor stop sequence         |               |

## Command Input (DISABLE)

| Coil Number | Function     | Description                              | DISABLE   |
|-------------|--------------|------------------------------------------|-----------|
| 10          | led_red      | Activate red LED                         |True       |
| 11          | led_green    | Activate green LED                       |True       |
| 12          | led_blue     | Activate blue LED                        |True       |
| 13          | led_off      | Turn off all LEDs                        | True      |
| 14          | led_white        | Activate all LEDs (white)               | True      |
| 15          | led_purple       | Activate red and green LEDs (purple)     |True       |
| 16          | led_yellow       | Activate red and blue LEDs (yellow)      |True       |
| 17          | led_indigo       | Activate green and blue LEDs (indigo)    |True       |