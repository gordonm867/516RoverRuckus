/*
package org.firstinspires.ftc.teamcode.Practices.Removed;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFHardware;

@TeleOp(name="InTestTwo", group="GOF")
@Disabled

public class InTest extends LinearOpMode {
    GOFHardware robot = new GOFHardware();
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            robot.setInPos((int)(100 * gamepad1.left_stick_y), 1);
            telemetry.addData("Motor Position: ", "" + robot.inWheel.getCurrentPosition());
            telemetry.addData("\nIntended Motor Position: ", (int)(100 * gamepad1.left_stick_y));
            telemetry.update();
        }
    }
}
*/

