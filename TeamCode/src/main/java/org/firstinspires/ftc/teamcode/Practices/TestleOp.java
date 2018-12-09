package org.firstinspires.ftc.teamcode.Practices;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GOFHardware;

@TeleOp(name="GOFTestleOp",group="GOFTests")
public class TestleOp extends LinearOpMode {

    GOFHardware robot = GOFHardware.getInstance();
    double[] point = new double[2];
    double angleOffset = 3;

    public void runOpMode() {
        point[0] = -2;
        point[1] = 2;
        boolean aPressed = false;
        boolean bPressed = false;
        boolean xPressed = false;
        boolean yPressed = false;
        boolean rbPressed = false;
        boolean lbPressed = false;
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a && !gamepad1.start) {
                aPressed = true;
            }
            if(gamepad1.b) {
                bPressed = true;
            }
            if(gamepad1.x) {
                xPressed = true;
            }
            if(gamepad1.y) {
                yPressed = true;
            }
            if(gamepad1.right_bumper) {
                rbPressed = true;
            }
            if(gamepad1.left_bumper) {
                lbPressed = true;
            }
            if(aPressed && !(gamepad1.a && !gamepad1.start)) {
                aPressed = false;
                turn(90, 30);
            }
            if(bPressed && !gamepad1.b) {
                bPressed = false;
                turn(-90, 30);
            }
            if(xPressed && !gamepad1.x) {
                xPressed = false;
                runToPoint(-4, 0);
            }
            if(yPressed && !gamepad1.y) {
                yPressed = false;
                encoderMovePreciseTimed(0, 1, 1);
            }
            if(rbPressed && !gamepad1.right_bumper) {
                rbPressed = false;
                encoderMovePreciseTimed(-(int)(12 * 560 / (4 * Math.PI)), 1, 1);
            }
            if(lbPressed && !gamepad1.left_bumper) {
                lbPressed = false;
                encoderMovePreciseTimed((int)(12 * 560 / (4 * Math.PI)), 1, 1);
            }
        }
    }

    private double atan(double num) { // Returns atan in degrees
        return(Math.atan(num) * (180 / Math.PI));
    }

    private void runToPoint(double x, double y) {
        double yDiff = point[1] - y;
        double xDiff = point[0] - x;
        double straightAngle = atan(yDiff / xDiff);
        Orientation g0angles = null;
        Orientation g1angles = null;
        double robotAngle;
        if (robot.gyro0 != null) {
            g0angles = robot.gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // Get z axis angle from first gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
        }
        if (robot.gyro1 != null) {
            g1angles = robot.gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // Get z axis angle from second gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
        }
        if (g0angles != null && g1angles != null) {
            robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2); // Average angle measures to determine actual robot angle
        } else if (g0angles != null) {
            robotAngle = g0angles.firstAngle;
        } else if (g1angles != null) {
            robotAngle = g1angles.firstAngle;
        } else {
            robotAngle = 0;
        }
        double turnAngle = straightAngle - (robotAngle - 45);
        if(turnAngle > 360) {
            turnAngle -= 360;
        }
        if(turnAngle > 180) {
            turnAngle -= 180;
        }
        if(turnAngle < -360) {
            turnAngle += 360;
        }
        if(turnAngle < -180) {
            turnAngle += 360;
        }
        boolean backward;
        if(Math.min(Math.abs(turnAngle), Math.abs(180 + turnAngle)) == Math.abs(turnAngle)) {
            turn(turnAngle, 3);
            backward = false;
        }
        else {
            turn(180 + turnAngle, 3);
            backward = true;
        }
        while(!gamepad1.a) {}
        double distance;
        if(!backward) {
            distance = -((12 * Math.hypot(xDiff, yDiff)) - 9);
        }
        else {
            distance = ((12 * Math.hypot(xDiff, yDiff)) - 9);
        }
        while(opModeIsActive() && !gamepad1.a) {}
        encoderMovePreciseTimed((int)(distance * 560 / (4 * Math.PI)), 0.35, (distance * 560 / (4 * Math.PI)) / 1500);
        while(opModeIsActive() && !gamepad1.a) {}
        point[0] = x;
        point[1] = y;
    }

    private void encoderMovePreciseTimed(int pos, double speed, double timeLimit) { // Move encoders towards target position until the position is reached, or the time limit expires
        if (opModeIsActive() && robot.rrWheel != null && robot.rfWheel != null && robot.lrWheel != null && robot.lfWheel != null && robot.hangOne != null) {
            robot.rrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            idle();
            robot.rrWheel.setTargetPosition(pos);
            robot.rfWheel.setTargetPosition(pos);
            robot.lfWheel.setTargetPosition(pos);
            robot.lrWheel.setTargetPosition(pos);
            try {
                robot.setDrivePower(-(pos / Math.abs(pos)) * speed, -(pos / Math.abs(pos)) * speed, -(pos / Math.abs(pos)) * speed, -(pos / Math.abs(pos)) * speed);
            }
            catch(Exception p_exception) {
                robot.setDrivePower(-(robot.lrWheel.getCurrentPosition() / Math.abs(robot.lrWheel.getCurrentPosition())), -(robot.lfWheel.getCurrentPosition() / Math.abs(robot.lfWheel.getCurrentPosition())), -(robot.rrWheel.getCurrentPosition() / Math.abs(robot.rrWheel.getCurrentPosition())), -(robot.rfWheel.getCurrentPosition() / Math.abs(robot.rfWheel.getCurrentPosition())));
            }
            ElapsedTime limitTest = new ElapsedTime();
            while ((robot.rrWheel.isBusy() || robot.rfWheel.isBusy() || robot.lrWheel.isBusy() || robot.lfWheel.isBusy()) && opModeIsActive() && limitTest.time() < timeLimit) {
                updateTelemetry();
            }
            if(limitTest.time() > timeLimit) {
                robot.rrWheel.setTargetPosition((robot.rrWheel.getCurrentPosition()));
                robot.rfWheel.setTargetPosition((robot.rfWheel.getCurrentPosition()));
                robot.lrWheel.setTargetPosition((robot.lrWheel.getCurrentPosition()));
                robot.lfWheel.setTargetPosition((robot.lfWheel.getCurrentPosition()));
            }
            robot.setDrivePower(0, 0, 0, 0);
            resetEncoders();
            sleep(100);
        }
    }

    private void turn(double angle, double time) {
        angle += 3;
        double angleIntended;
        double robotAngle;
        ElapsedTime turnTime = new ElapsedTime();
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotAngle = getAngle();
        angleIntended = robotAngle + angle;
        if (angleIntended < robotAngle) { // Left turn
            if(angleIntended > 180) {
                angleIntended -= 360;
            }
            else if(angleIntended < -180) {
                angleIntended += 360;
            }
            telemetry.addData("Status", "Turning to angle " + angleIntended);
            telemetry.update();
            while(opModeIsActive() && !(angleIntended - angleOffset < robotAngle && angleIntended + angleOffset > robotAngle) && turnTime.time() < time) {
                double error = Math.abs(robotAngle - angleIntended);
                robot.setDrivePower(Math.min(-0.01 * error, -0.1), Math.min(-0.025 * error, -0.1), Math.max(0.01 * error, 0.1), Math.max(0.01 * error, 0.1));
                robotAngle = getAngle();
                telemetry.addData("Angle", robotAngle);
                telemetry.addData("Error", robotAngle - angleIntended);
                telemetry.update();
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        else if(opModeIsActive() && angleIntended > robotAngle) { // Right turn
            if(angleIntended > 180) {
                angleIntended -= 360;
            }
            else if(angleIntended < -180) {
                angleIntended += 360;
            }
            telemetry.addData("Status", "Turning to angle " + angleIntended);
            telemetry.update();
            while(opModeIsActive() && !(angleIntended - angleOffset < robotAngle && angleIntended + angleOffset > robotAngle) && turnTime.time() < time) {
                double error = Math.abs(robotAngle - angleIntended);
                robot.setDrivePower(Math.max(0.01 * error, 0.1), Math.max(0.01 * error, 0.1), Math.min(-0.01 * error, -0.1), Math.min(-0.01 * error, -0.1));
                robotAngle = getAngle();
                telemetry.addData("Angle", robotAngle);
                telemetry.addData("Error", angleIntended - robotAngle);
                telemetry.update();
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        resetEncoders();
        double error = angleIntended - robotAngle;
        if(time - turnTime.time() > 0 && Math.abs(error) > angleOffset && opModeIsActive()) {
            turn(error, time - turnTime.time());
        }
    }

    private double getAngle() {
        double robotAngle;
        Orientation g0angles = null;
        Orientation g1angles = null;
        if (robot.gyro0 != null) {
            g0angles = robot.gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // Get z axis angle from first gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
        }
        if (robot.gyro1 != null) {
            g1angles = robot.gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // Get z axis angle from second gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
        }
        if (g0angles != null && g1angles != null) {
            robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2); // Average angle measures to determine actual robot angle
        } else if (g0angles != null) {
            robotAngle = g0angles.firstAngle;
        } else if (g1angles != null) {
            robotAngle = g1angles.firstAngle;
        } else {
            robotAngle = 0;
        }
        return robotAngle;
    }

    private void resetEncoders() { // Reset encoder values and set encoders to "run to position" mode
        robot.rrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        idle();
    }

    private void updateTelemetry() { // Update telemetry on autonomous statuses
        String telemetryString = "";
        telemetryString += "Remaining Distances\n";
        telemetryString += "  rr: " + (robot.rrWheel.getTargetPosition() - robot.rrWheel.getCurrentPosition());
        telemetryString += "  rf: " + (robot.rfWheel.getTargetPosition() - robot.rfWheel.getCurrentPosition());
        telemetryString += "  lr: " + (robot.lrWheel.getTargetPosition() - robot.lrWheel.getCurrentPosition());
        telemetryString += "  lf: " + (robot.lfWheel.getTargetPosition() - robot.lfWheel.getCurrentPosition());
        telemetryString += "  h1: " + (robot.hangOne.getTargetPosition() - robot.hangOne.getCurrentPosition());
        Orientation g0angles = null;
        Orientation g1angles = null;
        if (robot.gyro0 != null) {
            g0angles = robot.gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from first gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
        }
        if (robot.gyro1 != null) {
            g1angles = robot.gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from second gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
        }
        double robotAngle;
        if (g0angles != null && g1angles != null) {
            robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2); // Average angle measures to determine actual robot angle
        } else if(g0angles != null) {
            robotAngle = g0angles.firstAngle;
        } else if(g1angles != null) {
            robotAngle = g1angles.firstAngle;
        } else {
            robotAngle = 0;
        }
        telemetryString += ("Robot angle: " + (robotAngle - 45));
        telemetry.addData("", telemetryString);
        telemetry.update();
    }
}
