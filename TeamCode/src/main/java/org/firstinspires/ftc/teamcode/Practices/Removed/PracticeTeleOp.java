/* package org.firstinspires.ftc.teamcode.Practices.Removed;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.util.Range.clip;

/*
 * This file is intended to enable basic motion controls for a robot with two motors.
 */

/**
 *          Alternative Experimental Code #2 - see this to learn how NOT to code
 *          ============================================================================================
 *          if((gamepad1.left_stick_x > 0.1) || (gamepad1.left_stick_x < -0.1)) {
 *              if(gamepad1.left_stick_x < -0.1) {
 *                  leftBackDrive.setPower(leftFrontPower * -1);
 *                  rightBackDrive.setPower(rightFrontPower);
 *              }
 *              if(gamepad1.left_stick_x > 0.1) {
 *                  rightBackDrive.setPower(rightFrontPower * -1);
 *                  leftBackDrive.setPower(leftFrontPower);
 *              }
 *          }
 *          else {
 *              leftBackDrive.setPower(leftFrontPower);
 *              rightBackDrive.setPower(rightFrontPower);
 *          }
 *
 *          telemetry.addData("Status", "Run Time: " + runtime.toString());
 *          telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
 *          telemetry.update();
 *          ============================================================================================
 *
 **/

/* @TeleOp(name="Linear OpMode Test", group="Linear OpMode")
@Disabled

public class PracticeTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    public void runOpMode() {

        // Introduce local variables

        // Define each motor ("lf" = left-front, "lb" = left-back, "rf" = right-front, "rb" = right-back)
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");

        // Set each motor direction; the right motors should move backwards and the left should move forwards to move the robot correctly
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the phone that the robot has been initialized successfully
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the "start" command, and reset the timer
        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {

            // Define variables to store the value of the power, from -1 to 1, which will be sent to each motor
            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;

            double drive = -gamepad1.left_stick_y; // Set drive variable based on vertical position of left game pad stick
            double turn  =  gamepad1.right_stick_x; // Set turn variable based on horizontal position of right game pad stick

            if(gamepad1.dpad_down || gamepad1.dpad_up) {
                if(gamepad1.left_stick_y != 0) {
                    drive = drive * 0.25; // If the dpad buttons are being used in addition to the joystick, reduce the drive variable to 1/4 of the joystick position
                }
                else {
                    if(gamepad1.dpad_down) {
                        drive = -0.25; // If the dpad's down button is being used without the joystick, set the drive variable to -0.25.
                    }
                    else {
                        drive = 0.25; // If the dpad's up button is being used without the joystick, set the drive variable to 0.25.
                    }
                }
            }

            if(gamepad1.dpad_right || gamepad1.dpad_left) { // Do the same thing for the turn variable as with the drive variable above
                if(gamepad1.right_stick_x != 0) {
                    turn = turn * 0.25;
                }
                else {
                    if(gamepad1.dpad_left) {
                        turn = -0.25;
                    }
                    else {
                        turn = 0.25;
                    }
                }
            }

            // Calculate each motor power, clipping each value to ensure that no value below -1 or above 1 is sent
            leftFrontPower  = clip(drive + turn + gamepad1.left_stick_x, -1.0, 1.0);
            rightFrontPower = clip(drive - turn - gamepad1.left_stick_x, -1.0, 1.0);
            rightBackPower = clip(drive - turn + gamepad1.left_stick_x, -1.0, 1.0);
            leftBackPower = clip(drive + turn - gamepad1.left_stick_x, -1.0, 1.0);

            // Send the powers to the motors
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Give more information to the phone
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left front (%.2f), right front (%.2f), left back (%.2f), right back (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.update();

        }
    }
} */