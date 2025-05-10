# Coil Mapping Reference

This code is referenced from an older version. Remove references to `touchless_switch` and `Alcohol`.

# Change log

`update 10-05-2568`
* **Disable LED Command ALL** because it is no longer in use
* **Change Emer to realworld** Reversed the behavior of the 'Emer' switch: true now represents false, and false represents true

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
| 10          | led_red      | Activate red LED                         |True       |
| 11          | led_green    | Activate green LED                       |True       |
| 12          | led_blue     | Activate blue LED                        |True       |
| 13          | led_off      | Turn off all LEDs                        | True      |
| 14          | led_white        | Activate all LEDs (white)               | True      |
| 15          | led_purple       | Activate red and green LEDs (purple)     |True       |
| 16          | led_yellow       | Activate red and blue LEDs (yellow)      |True       |
| 17          | led_indigo       | Activate green and blue LEDs (indigo)    |True       |
| 20          | start_motor  | Initiate the motor start sequence        |               |
| 21          | stop_motor   | Initiate the motor stop sequence         |               |
