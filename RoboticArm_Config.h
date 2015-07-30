#pragma once
/*
 * Please use the following file for any pin or configuration
 * definitions, this will be dependency injected into the source
 * code that makes use of the following static members.
 *
 * The following is Intel's Galileo layout for the pins.
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
 * Note: Galileo's cannot go slower than ~125 Hz on Linux SYSFS PWM.
 * 
 * The quadrature encoders in this project are a Pololu brand ones.
 * one offers 48 CPR in a 75:1 configuration, while the other is a
 * 64 CPR in a 29:1 configuration. (CPR = Counts Per Revolution).
 * 
 *
 *
 *
 * References:
 * http://www.malinov.com/Home/sergey-s-blog/intelgalileo-programminggpiofromlinux
 * https://www.pololu.com/product/1443
 * https://www.pololu.com/product/2286
 *
 */


namespace config
{
    /* The physical length of each of the links in meters */
    static constexpr float link_lengths[] = { 0.015, 0.015 };

    /* Pair of pins used for these elements */
    static constexpr int quad_enc_pins[][2] = {{24, 25}, {26, 27}};
    static constexpr int dc_motor_pins[][2] = {{ 3,  7}, { 1,  5}};
    
    /* All of the joints will utilize the same webcam port in this case */
    static constexpr int visual_enc_ports[] = {0, 0};
    
    /* Physical characteristics of the encoders being used */
    static constexpr long quad_enc_segments[] = {64 * 29, 48 * 75};

    /* Calculate number of joints based of motors */
    static const int joints_nr = sizeof(dc_motor_pins)/sizeof(dc_motor_pins[0]);
}

