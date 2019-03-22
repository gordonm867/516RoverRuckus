/*
package org.firstinspires.ftc.teamcode.Practices.Removed;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GOFHardware;

public class GOFLightsTest extends LinearOpMode {

    GOFHardware robot = new GOFHardware();

    private boolean xpressed = false;
    private boolean ypressed = false;
    private double lightpow = 0;

    public void runOpMode() {

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.x && !xpressed) {
                xpressed = true;
            } else if (!gamepad1.x) {
                xpressed = false;
            }

            if (gamepad1.y && !ypressed) {
                ypressed = true;
            } else if (!gamepad1.y) {
                ypressed = false;
            }


            if (xpressed) { // LED test
                if (lightpow < 1) {
                    while (gamepad1.x) {}
                    lightpow += 0.25;
                }
                robot.setLightPower(lightpow);
            }

            if (ypressed) {
                if (lightpow > 0) {
                    while (gamepad1.y) {}
                    lightpow -= 0.25;
                }
                robot.setLightPower(lightpow);
            }
        }
    }
} */