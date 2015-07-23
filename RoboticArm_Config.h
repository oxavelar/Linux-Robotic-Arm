#pragma once
/*
 * Please use the following file for any pin or configuration
 * definitions, this will be dependency injected into the source
 * code that makes use of the following static members.
 *
 *  The following is Intel's Galileo layout for the pins.
 *
 *   +==========+=========+=======================+
 *   |  SYS_FS  |   LABEL |           DESCRIPTION |
 *   +==========+=========+=======================+
 *   |  gpio24  |     IO6 |       QE Channel A #1 |
 *   |  gpio25  |    IO11 |       QE Channel B #1 |
 *   |  gpio26  |     IO8 |       QE Channel A #2 |
 *   |  gpio27  |     IO7 |       QE Channel B #2 |
 *   |    pwm3  |     IO3 |   Motor DC Ctrl CW #1 |
 *   |    pwm7  |    IO10 |  Motor DC Ctrl CCW #1 |
 *   |    pwm1  |     IO9 |   Motor DC Ctrl CW #2 |
 *   |    pwm5  |     IO5 |  Motor DC Ctrl CCW #2 |
 *   |  video0  | USB HST |         USB HD WebCam |
 *   +==========+=========+====================== +
 *
 *  Note: Galileo's cannot go slower than ~125 Hz on Linux SYSFS PWM.
 *
 * References:
 * http://www.malinov.com/Home/sergey-s-blog/intelgalileo-programminggpiofromlinux
 */


namespace config
{
    /* Pair of pins used for these elements */
    static constexpr int quad_enc_pins[][2] = {{24, 25}, {26, 27}};
    static constexpr int dc_motor_pins[][2] = {{ 3,  7}, { 1,  5}};
    
    /* All of the joints will utilize the same webcam port in this case */
    static constexpr int visual_enc_ports[] = {0, 0};
    
    /* Calculate number of joints based of motors */
    static const int joints_nr = sizeof(dc_motor_pins)/sizeof(dc_motor_pins[0]);
}
