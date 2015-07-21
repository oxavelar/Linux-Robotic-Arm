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
 *   |      24  |     IO6 |       QE Channel A #1 |
 *   |      25  |    IO11 |       QE Channel B #1 |
 *   |      26  |     IO8 |       QE Channel A #2 |
 *   |      27  |     IO7 |       QE Channel B #2 |
 *   |       3  |    PWM3 |      Motor DC Ctrl #1 |
 *   |      15  |     IO3 | H-Bridge direction #1 |
 *   |       9  |    PWM1 |      Motor DC Ctrl #2 |
 *   |      17  |     IO5 | H-Bridge direction #2 |
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
    static constexpr int quad_enc_pins[][2] = {{24, 25}, {27, 26}};
    static constexpr int dc_motor_pins[][2] = {{ 3, 15}, { 1, 17}};
    
    /* Calculate number of joints based of motors */
    static const int joints_nr = sizeof(dc_motor_pins)/sizeof(dc_motor_pins[0]);
}

