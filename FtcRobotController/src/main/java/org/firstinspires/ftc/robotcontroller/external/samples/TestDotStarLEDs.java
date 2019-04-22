package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "TestDotStarLEDs", group = "Test")
public class TestDotStarLEDs extends OpMode {

    /* LEDs: Use this line if you drive the LEDs using an I2C/SPI bridge. */
    DotStarBridgedLED leds;

    /* Use this instead of the line above if you are driving the LEDs using digital outputs.
     * Warning: the color updates will be slow.
     */
    // DotStarLED leds;

    /* Run the LED update code in a separate thread to avoid blocking important tasks. */
    Thread led_thread;

    boolean xPressed = false;
    int x = 1;

    @Override
    public void init() {

        // Set up the LEDs. Change this to your configured name.
        leds = hardwareMap.get(DotStarBridgedLED.class, "leds");

        // Use ModernRoboticsDIM if using Modern Robotics hardware.
        leds.setController(DotStarBridgedLED.Controller.RevExpansionHub);

        // Set the length of the strip.
        leds.setLength(3);
    }

    public void start() {

        /* We use a separate thread for updating the LEDs. This allows us to continually change the
         * colors without worrying about delaying more important tasks, like driving.
         */

        led_thread = new Thread() {
            public void run() {

                // Continue until the program stops.
                while (true) {
                    if (Thread.interrupted()) return;

                    if (x == 1) {
                        leds.pixels[0].setRGB(48, 16, 0);  // Orange
                        leds.pixels[1].setRGB(40, 48, 48); // White
                        leds.pixels[2].setRGB(40, 48, 48); // White
                    } else if (x == 2) {
                        leds.pixels[0].setRGB(40, 48, 48); // White
                        leds.pixels[1].setRGB(48, 16, 0);  // Orange
                        leds.pixels[2].setRGB(40, 48, 48); // White
                    } else if (x == 3) {
                        leds.pixels[0].setRGB(40, 48, 48); // White
                        leds.pixels[1].setRGB(40, 48, 48); // White
                        leds.pixels[2].setRGB(48, 16, 0);  // Orange
                    } else if (x == 4) {
                        leds.pixels[0].setRGB(48, 0, 0);   // Red
                        leds.pixels[1].setRGB(0, 48, 0);   // Green
                        leds.pixels[2].setRGB(0, 0, 48);   // Blue
                    } else if (x == 5) {
                        leds.pixels[0].setRGB(48, 48, 0);  // Yellow
                        leds.pixels[1].setRGB(48, 0, 48);  // Purple
                        leds.pixels[2].setRGB(0, 48, 48);  // Teal
                    }else if (x == 6){
                        leds.pixels[0].setRGB(0, 0, 0);    // off
                        leds.pixels[1].setRGB(0, 0, 0);    // off
                        leds.pixels[2].setRGB(0, 0, 0);    // off
                    }

                    // Flush the current set of colors to the strip.
                    leds.update();
                }
            }
        };

        // Start the color updates.
        led_thread.start();
    }

    @Override
    public void loop() {

        if(gamepad1.x && !xPressed){
            xPressed = true;
            x++;
            if (x > 6){
                x=1;
            }
        }else if (!gamepad1.x && xPressed){
            xPressed = false;
        }

    }

    @Override
    public void stop() {

        // End the color updates.
        led_thread.interrupt();
    }
}