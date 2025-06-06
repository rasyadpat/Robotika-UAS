# <ins>**Robotics</ins> - Final Project**: Battlebot

__Telkom University__, Bandung, Indonesia

##### Group Members:
- 1102224262 - Sinarjonnan Prastiyaji Ariyanto
- 1102224278 - Lintar Alvisyahmulya Sukoco
- 1102224221 - Rasyad Rifatan Rahman Dingang Patianom

## Control Scheme

XBOX Series controller (XInput) is used for this project, although with the [Bluepad32](https://github.com/ricardoquesada/bluepad32) package, most bluetooth based controllers should work.

Right Trigger = Accelerate<br>
Left Trigger  = Decelerate<br>
Right Bumper  = Rotate Right<br>
Left Bumper   = Rotate Left<br>
Right Stick   = Steering<br>


## To-Do

- Adjust and tinker with the steering value
- Maybe make the R2 and L2 priority system based on the throttle and brake values?
- Optimize int pwm to byte and other data types that might need optimization
- Optimize so that handleacceleration handles the default setspeed instead of handlesteering

## Dependencies

- [L298N Library](https://github.com/AndreaLombardo/L298N) by AndreaLombardo
- [Bluepad32 Package](https://github.com/ricardoquesada/bluepad32) by ricardoquesada