# Linux-Robotic-Arm

The demo and application models a RR arm on a Intel Galileo2 system running Linux.

It makes use of inverse and forward kinematics calculations on different position sensors in order to obtain the rotor angular state, and then spawns unique control threads per joint in order to keep the joints to the reference/target angle value.


### Layered SW Architecture
<img align="center" src="http://imgh.us/SW_Arch_2.svgz">

### Class Structure
A robot joint is formed by a positioning (imaging/encoder) and movement (actuator/motor) objects, by having this abstraction we can make a robotic arm operate with different layers and or objects.
<img align="center" src="http://imgh.us/SW_Joint.svgz">

Contributors:
Omar X. Avelar
Juan C. Razo
