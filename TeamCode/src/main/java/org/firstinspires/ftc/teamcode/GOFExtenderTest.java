package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="GOFExtenderTest",group="GOFTests")
public class GOFExtenderTest extends LinearOpMode {
    public void runOpMode() {
        GOFHardware robot = GOFHardware.getInstance();
        robot.init(hardwareMap);
        robot.extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        boolean aPressed = false;
        boolean bPressed = false;
        boolean xPressed = false;
        boolean yPressed = false;
        while(opModeIsActive()) {
            if (gamepad1.a && !gamepad1.start) {
                aPressed = true;
            }
            if (gamepad1.b) {
                bPressed = true;
            }
            if (gamepad1.x) {
                xPressed = true;
            }
            if (gamepad1.y) {
                yPressed = true;
            }
            if (aPressed && !(gamepad1.a && !gamepad1.start)) {
                robot.extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.extend.setPower(-0.3);
                sleep(500);
                robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.extend.setTargetPosition(robot.extend.getCurrentPosition());
            }
            if (bPressed && !gamepad1.b) {
                robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.extend.setPower(0.6);
                robot.extend.setTargetPosition(500);
                while(robot.extend.isBusy() && opModeIsActive()) {}
                robot.extend.setTargetPosition(robot.extend.getCurrentPosition());
                robot.extend.setPower(0.3);
            }
            if (xPressed && !gamepad1.x) {
                robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.extend.setPower(0.6);
                robot.extend.setTargetPosition(1000);
                while(robot.extend.isBusy() && opModeIsActive()) {}
                robot.extend.setTargetPosition(robot.extend.getCurrentPosition());
                robot.extend.setPower(0.3);
            }
            if (yPressed && !gamepad1.y) {
                robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.extend.setPower(0.6);
                robot.extend.setTargetPosition(0);
                while(robot.extend.isBusy() && opModeIsActive()) {}
                robot.extend.setTargetPosition(robot.extend.getCurrentPosition());
                robot.extend.setPower(0.3);
            }
        }
    }
}
