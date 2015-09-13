#pragma once
/*
 * Please use the following file for any pin or configuration
 * definitions, this will be dependency injected into the source
 * code that makes use of the following static members.
 *
 * The following is Intel's Edison layout for the pins.
 *
 *   +===========+=========+=======================+==========
 *   |   SYS_FS  |   LABEL |           DESCRIPTION |   COLOR |
 *   +===========+=========+=======================+==========
 *   |   gpio49  |     IO8 |       QE Channel A #1 |   White |
 *   |   gpio48  |     IO7 |       QE Channel B #1 |  Yellow |
 *   |   gpio41  |    IO10 |       QE Channel A #2 |   White |
 *   |   gpio43  |    IO11 |       QE Channel B #2 |  Yellow |
 *   |     pwm0  |     IO3 |   Motor DC Ctrl CW #1 |     Red |
 *   |     pwm2  |     IO6 |  Motor DC Ctrl CCW #1 |    Blue |
 *   |     pwm3  |     IO9 |   Motor DC Ctrl CW #2 |     Red |
 *   |     pwm1  |     IO5 |  Motor DC Ctrl CCW #2 |    Blue |
 *   |   video0  | USB HST |         USB HD WebCam |         |
 *   +===========+=========+====================== +=========+
 *
 * Note: Pololu encoders cable colors match the ones above.
 * 
 * The quadrature encoders in this project are a Pololu brand ones.
 * one offers 48 CPR in a 75:1 configuration, while the other is a
 * 64 CPR in a 29:1 configuration. (CPR = Counts Per Revolution).
 * 
 *
 *
 *
 * References:
 * http://www.emutexlabs.com/project/215-intel-edison-gpio-pin-multiplexing-guide
 * http://www.malinov.com/Home/sergey-s-blog/intelgalileo-programminggpiofromlinux
 * https://www.pololu.com/product/1443
 * https://www.pololu.com/product/2286
 *
 */


namespace config
{
    /* The rate at which the quadrature encoder operates */
    static constexpr int quad_encoder_rate = 4;

    /* The physical length of each of the links in meters */
    static constexpr double link_lengths[] = { 0.015, 0.015 };

    /* Pair of pins used for these elements */
    static constexpr int quad_encoder_pins[][2]  = {{ 49,  48}, { 41,  43}};
    static constexpr int dc_motor_pins[][2]      = {{  0,   2}, {  3,   1}};
    
    /* All of the joints will utilize the same webcam port in this case */
    static constexpr int visual_encoder_ports[] = {0, 0};
    
    /* Physical characteristics of the encoders being used */
    static constexpr long quad_encoder_segments[] = {64 * 29, 48 * 75};

    /* Calculate number of joints based of motors */
    static constexpr int joints_nr = sizeof(dc_motor_pins)/sizeof(dc_motor_pins[0]);
}

