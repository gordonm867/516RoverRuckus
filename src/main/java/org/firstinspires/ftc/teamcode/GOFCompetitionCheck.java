package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="GOFCompetitionCheck",group="GOFPreparation")
public class GOFCompetitionCheck extends LinearOpMode {

    GOFHardware robot = GOFHardware.getInstance();

    public void runOpMode() {

        robot.init(hardwareMap);
        waitForStart();

        boolean functional = true;

        if(robot.rrWheel.getCurrentPosition() == 0) { // Checks position of the right rear wheel
            if (!checkMotor(robot.rrWheel)) {
                telemetry.addData("Note", "Right rear wheel is not working; please check general functionality as well as encoder wiring");
                functional = false;
            }
        }
        if(robot.rfWheel.getCurrentPosition() == 0) { // Checks the position of the right front wheel
            if (!checkMotor(robot.rfWheel)) {
                telemetry.addData("Note", "Right front wheel is not working; please check general functionality as well as encoder wiring");
                functional = false;
            }
        }
        if(robot.lfWheel.getCurrentPosition() == 0) { // Checks the position of the left front wheel
            if (!checkMotor(robot.lfWheel)) {
                telemetry.addData("Note", "Left front wheel is not working; please check general functionality as well as encoder wiring");
                functional = false;
            }
        }
        if(robot.lrWheel.getCurrentPosition() == 0) { // Checks the position of the right real wheel
            if (!checkMotor(robot.lrWheel)) {
                telemetry.addData("Note", "Left rear wheel is not working; please check general functionality as well as encoder wiring");
                functional = false;
            }
        }
        if(robot.hangOne.getCurrentPosition() == 0) { // Checks the position of the of Hang Wheel One
            if (!checkMotor(robot.hangOne)) {
                telemetry.addData("Note", "Hang wheel is not working; please check general functionality as well as encoder wiring");
                functional = false;
            }
        }
        if(robot.intake.getCurrentPosition() == 0) { // Checks if the position of the intake is equal to 0
            if (!checkMotor(robot.lrWheel)) {
                telemetry.addData("Note", "Intake is not working; please check general functionality as well as encoder wiring");
                functional = false;
            }
        }
        if(!checkServo(robot.kicker)) { // Moves servo to see if it is moving
            telemetry.addData("Note", "Kicker servo is not working; please check general functionality");
            functional = false;
        }

        if(functional) {
            telemetry.addData("Note", "Everything is working fine!") ;
        }

        telemetry.update();
    }

    public boolean checkMotor(DcMotor wheel) {
        wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel.setTargetPosition(10);
        wheel.setPower(0.5);
        sleep(25);
        wheel.setPower(0);
        if(wheel.getCurrentPosition() == 0) {
            if(wheel == robot.hangOne) {
                wheel.setTargetPosition(-10);
                wheel.setPower(0.5);
                sleep(25);
                wheel.setPower(0);
            }
            wheel.setTargetPosition(wheel.getCurrentPosition());
            return false;
        }
        else {
            wheel.setTargetPosition(wheel.getCurrentPosition());
            return true;
        }
    }

    public boolean checkServo(Servo servo) {
        double initPos = servo.getPosition();
        if(servo.getPosition() + 0.05 < 1) {
            servo.setPosition(servo.getPosition() + 0.05);
        }
        else {
            servo.setPosition(servo.getPosition() + 0.05);
        }
        sleep(25);
        if(servo.getPosition() == initPos) {
            return false;
        }
        else {
            return true;
        }
    }
}
