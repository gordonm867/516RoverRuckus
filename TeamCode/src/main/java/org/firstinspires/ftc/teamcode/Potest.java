package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Potest",group="GOFTests")
// @Disabled
public class Potest extends LinearOpMode {
    GOFHardware robot = GOFHardware.getInstance();
    public void runOpMode() {
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.left_stick_y == 0) {
                robot.box.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.box.setPower(0); // Hi
            }
            else {
                robot.box.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.box.setPower(gamepad1.left_stick_y);
            }
            telemetry.addData("Angle", ((robot.boxPotentiometer.getVoltage() / 3.3) * 180));
            telemetry.update();
            robot.setInPower(-gamepad1.right_stick_y);
        }
    }
}
