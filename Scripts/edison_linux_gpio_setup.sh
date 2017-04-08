# The following is required in order to mux correctly our 4 PWM channels
# and 2 GPIO pins used in our Edison Board.
#
# These are used by our robot to control the joints position and movement
# detection. According to the website below you must enable the PWM channel
# with muxing pins, and depending the mode is either PWM, GPIO, SPI, ADC
# amongs others. Please make sure to read the website below for better under-
# -standing.
#
#
# http://www.emutexlabs.com/project/215-intel-edison-gpio-pin-multiplexing-guide

echo "I: Intel Edison IO Pre-Configuration"

# Setting tri-state to make IO changes
echo 214 > /sys/class/gpio/export
echo low > /sys/class/gpio/gpio214/direction

echo "I: Enabling PWM0 on IO3"
echo 251 > /sys/class/gpio/export
echo 219 > /sys/class/gpio/export
echo high > /sys/class/gpio/gpio251/direction
echo in > /sys/class/gpio/gpio219/direction
echo mode1 > /sys/kernel/debug/gpio_debug/gpio12/current_pinmux
echo no-override > /sys/kernel/debug/gpio_debug/gpio12/current_override_indir
echo override-enable > /sys/kernel/debug/gpio_debug/gpio12/current_override_outdir
echo 251 > /sys/class/gpio/unexport
echo 219 > /sys/class/gpio/unexport

echo "I: Enabling PWM1 on IO5"
echo 253 > /sys/class/gpio/export
echo 221 > /sys/class/gpio/export
echo high > /sys/class/gpio/gpio253/direction
echo in > /sys/class/gpio/gpio221/direction
echo mode1 > /sys/kernel/debug/gpio_debug/gpio13/current_pinmux
echo no-override > /sys/kernel/debug/gpio_debug/gpio13/current_override_indir
echo override-enable > /sys/kernel/debug/gpio_debug/gpio13/current_override_outdir
echo 253 > /sys/class/gpio/unexport
echo 221 > /sys/class/gpio/unexport

echo "I: PWM1 Noise workaround"
echo 1 > /sys/class/pwm/pwmchip0/export
echo 1 > /sys/class/pwm/pwmchip0/pwm1/enable
echo 0 > /sys/class/pwm/pwmchip0/pwm1/enable
echo 1 > /sys/class/pwm/pwmchip0/unexport

echo "I: Enabling PWM2 on IO6"
echo 254 > /sys/class/gpio/export
echo 222 > /sys/class/gpio/export
echo high > /sys/class/gpio/gpio254/direction
echo in > /sys/class/gpio/gpio222/direction
echo mode1 > /sys/kernel/debug/gpio_debug/gpio182/current_pinmux
echo no-override > /sys/kernel/debug/gpio_debug/gpio182/current_override_indir
echo override-enable > /sys/kernel/debug/gpio_debug/gpio182/current_override_outdir
echo 254 > /sys/class/gpio/unexport
echo 222 > /sys/class/gpio/unexport

echo "I: Enabling PWM3 on IO9"
echo 257 > /sys/class/gpio/export
echo 225 > /sys/class/gpio/export
echo high > /sys/class/gpio/gpio257/direction
echo in > /sys/class/gpio/gpio225/direction
echo mode1 > /sys/kernel/debug/gpio_debug/gpio183/current_pinmux
echo no-override > /sys/kernel/debug/gpio_debug/gpio183/current_override_indir
echo override-enable > /sys/kernel/debug/gpio_debug/gpio183/current_override_outdir
echo 257 > /sys/class/gpio/unexport
echo 225 > /sys/class/gpio/unexport

echo "I: Enabling GPIO48 on IO7"
echo mode0 > /sys/kernel/debug/gpio_debug/gpio48/current_pinmux
echo nopull > /sys/kernel/debug/gpio_debug/gpio48/current_pullmode
echo override-enable > /sys/kernel/debug/gpio_debug/gpio48/current_override_indir
echo no-override > /sys/kernel/debug/gpio_debug/gpio48/current_override_outdir

echo "I: Enabling GPIO49 on IO8"
echo mode0 > /sys/kernel/debug/gpio_debug/gpio49/current_pinmux
echo nopull > /sys/kernel/debug/gpio_debug/gpio49/current_pullmode
echo override-enable > /sys/kernel/debug/gpio_debug/gpio49/current_override_indir
echo no-override > /sys/kernel/debug/gpio_debug/gpio49/current_override_outdir


echo "I: Enabling GPIO41 on IO10"
echo 263 > /sys/class/gpio/export
echo 240 > /sys/class/gpio/export
echo 258 > /sys/class/gpio/export
echo 226 > /sys/class/gpio/export
echo high > /sys/class/gpio/gpio263/direction
echo low > /sys/class/gpio/gpio240/direction
echo mode0 > /sys/kernel/debug/gpio_debug/gpio41/current_pinmux
echo nopull > /sys/kernel/debug/gpio_debug/gpio41/current_pullmode
echo override-enable > /sys/kernel/debug/gpio_debug/gpio41/current_override_indir
echo no-override > /sys/kernel/debug/gpio_debug/gpio41/current_override_outdir
echo low > /sys/class/gpio/gpio258/direction
echo in > /sys/class/gpio/gpio226/direction
echo 263 > /sys/class/gpio/unexport
echo 240 > /sys/class/gpio/unexport
echo 258 > /sys/class/gpio/unexport
echo 226 > /sys/class/gpio/unexport

echo "I: Enabling GPIO43 on IO11"
echo 262 > /sys/class/gpio/export
echo 241 > /sys/class/gpio/export
echo 259 > /sys/class/gpio/export
echo 227 > /sys/class/gpio/export
echo high > /sys/class/gpio/gpio262/direction
echo low > /sys/class/gpio/gpio241/direction
echo mode0 > /sys/kernel/debug/gpio_debug/gpio43/current_pinmux
echo nopull > /sys/kernel/debug/gpio_debug/gpio43/current_pullmode
echo override-enable > /sys/kernel/debug/gpio_debug/gpio43/current_override_indir
echo no-override > /sys/kernel/debug/gpio_debug/gpio43/current_override_outdir
echo low > /sys/class/gpio/gpio259/direction
echo in > /sys/class/gpio/gpio227/direction
echo 262 > /sys/class/gpio/unexport
echo 241 > /sys/class/gpio/unexport
echo 259 > /sys/class/gpio/unexport
echo 227 > /sys/class/gpio/unexport

# Restoring back the IO's from tri-state
echo high > /sys/class/gpio/gpio214/direction
echo 214 > /sys/class/gpio/unexport

exit 0

