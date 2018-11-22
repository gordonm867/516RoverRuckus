package org.firstinspires.ftc.teamcode.Practices;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFHardware;

@TeleOp(name="GOFServoTest", group="GOFTests")
@Disabled

public class GOFServoTest extends LinearOpMode {

    public void runOpMode() {
        GOFHardware robot = new GOFHardware();
        robot.init(hardwareMap);
        waitForStart();

        double kickpow = 0;
        boolean rightBumperPressed = false;
        boolean leftBumperPressed = false;

        while(opModeIsActive()) {

            if (gamepad1.right_bumper) {
                rightBumperPressed = true;
            }

            if(rightBumperPressed && !gamepad1.right_bumper) {
                if (kickpow < 1) {
                    kickpow += 0.05;
                }
                robot.setKickPower(kickpow);
                rightBumperPressed = false;
            }

            if (gamepad1.left_bumper) {
                leftBumperPressed = true;
            }

            if(leftBumperPressed && !gamepad1.left_bumper) {
                if (kickpow > 0) {
                    kickpow -= 0.05;
                }
                robot.setKickPower(kickpow);
                leftBumperPressed = false;
            }
            telemetry.addData("Kicker position", "" + robot.kicker.getPosition());
            telemetry.update();
        }
    }
}