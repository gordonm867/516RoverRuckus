package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Iterator;
import java.util.List;

@Autonomous
@Disabled
public class CurvedAutoTest extends LinearOpMode {

    private                 ElapsedTime         elapsedTime             = new ElapsedTime(); // Measure timing
    private                 GOFHardware         robot                   = GOFHardware.getInstance();
    private                 GOFVuforiaLocalizer vuforia;

    private static final    String              TFOD_MODEL_ASSET        = "RoverRuckus.tflite";
    private static final    String              LABEL_GOLD_MINERAL      = "Gold Mineral";
    private static final    String              LABEL_SILVER_MINERAL    = "Silver Mineral";
    private static final    String              VUFORIA_KEY             = "AWVhzQD/////AAABmWz790KTAURpmjOzox2azmML6FgjPO5DBf5SHQLIKvCsslmH9wp8b5zkCGfES8tt+8xslwaK7sd2h5H1jwmix26x+Eg5j60l00SlNiJMDAp5IOMWvhdJGZ8jJ8wFHCNkwERQG57JnrOXVSFDlc1sfum3oH68fEd8RrA570Y+WQda1fP8hYdZtbgG+ZDVG+9XyoDrToYU3FYl3W" + "M1iUphAbHJz1BMFFnWJdbZzOicvqah/RwXqtxRDNlem3JdT4W95kCY5bckg92oaFIBk9n01Gzg8w5mFTReYMVI3Fne72/KpPRPJwblO0W9OI3o7djg+iPjxkKOeHUWW+tmi6r3LRaKTrIUfLfazRu0QwLA8Bgw";

    private                 TFObjectDetector    detector;

    private                 boolean             remove;
    private                 double              angleOffset             = 3;
    private                 double[]            point                   = new double[2];
    private                 double              startTime               = elapsedTime.time();
    private                 double              ticksPerInch            = 560 / (4 * Math.PI);
    private                 int                 goldPos                 = -2;

    private                 double              lfOffset;
    private                 double              lrOffset;

    public void runOpMode() {
        robot.init(hardwareMap);
        resetEncoders();
        int goldPos = detectGold();
        waitForStart();
        if(goldPos == 1) {
            robot.flipBox(0.456);
            robot.setInPos(72, 1);
            descend();
            encoderMovePreciseTimed(258, -392, -422, 358, 0.75, 1); // side to side
            resetEncoders();
            robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hangOne.setTargetPosition(-1560);
            robot.setHangPower(-1);
            turn(-getAngle(), 1);
            robot.flipBox(0.456);
            encoderMovePreciseTimed(-873, -646, -202, -846, 0.3, 1);
            resetEncoders();
            robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Status", "Turning " + -getAngle());
            telemetry.update();
            turn(-getAngle(), 1);
            robot.setInPower(0);
            robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turn(-135 - atan(-2, -0.118), 5);
            while (opModeIsActive() && !robot.bottomSensor.isPressed() && robot.hangOne.isBusy()) {
                double oldPos = robot.hangOne.getCurrentPosition();
                sleep(100);
                double newPos = robot.hangOne.getCurrentPosition();
                if (oldPos == newPos) {
                    break;
                }
            }
            robot.setHangPower(0);
            robot.flipBox(0.61);
            robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.extend.setTargetPosition(-1500);
            robot.extend.setPower(1);
            while (robot.extend.isBusy()) {}
            sample();
            robot.setInPower(1);
            robot.extend.setTargetPosition(-1800);
            robot.extend.setTargetPosition(0);
            sleep(3000);
            double x = -2;
            double maxX = -4.85;
            while (opModeIsActive() && Math.abs(x) < Math.abs(maxX)) {
                double currentY = Math.tan((2.0 / 3.0) * (x + 2)) + 2;
                double nextX = x - 0.1;
                double nextY = Math.tan((1.0 / 2.0) * (nextX + 2)) + 2;
                double angle = (Math.atan2(nextY - currentY, nextX - x));
                double side = Math.sin(angle);
                double forward = Math.cos(angle);
                while (Math.abs(((robot.lfWheel.getCurrentPosition() - lfOffset) - (robot.lrWheel.getCurrentPosition() - lrOffset)) / 2) <= (ticksPerInch / 12) * 0.1) {
                    robot.setDrivePower(forward - side, forward + side, forward + side, forward - side);
                }
                x = nextX;
            }
            robot.teamFlag.setPosition(0.920);
            turn(-getAngle() + 135, 8);
            point[0] = -4.85;
            point[1] = -4.81;
            runBackToPoint(-4.85, 2.9);
        }

    }

    private double atan(double y, double x) { // Returns atan in degrees
        return(Math.atan2(y, x) * (180 / Math.PI));
    }

    private void descend() {
        resetEncoders();
        robot.hangOne.setTargetPosition(1560);
        while (opModeIsActive() && robot.hangOne.getCurrentPosition() < 1560) {
            telemetry.addData("h1: ", "" + robot.hangOne.getCurrentPosition());
            telemetry.addData("Current angle", (getAngle() + 135));
            telemetry.update();
            robot.setHangPower(1);
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        robot.hangOne.setTargetPosition(robot.hangOne.getCurrentPosition()); // Set the target position to its current position to stop movement
        robot.setHangPower(0); // Stop sending power just in case
        resetEncoders();
        robot.hangOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset hang encoder
        robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set hang wheel back to run to position mode
    }

    private int detectGold() {
        while (!isStopRequested() && !isStarted()) {
            if (detector != null) { // The detector will be null if it's not supported on the device, which shouldn't be a concern, but this helps guarantee no crashes
                Recognition[] sampleMinerals = new Recognition[2];
                List<Recognition> updatedRecognitions = detector.getUpdatedRecognitions(); // ArrayList of detected objects
                if (updatedRecognitions != null) { // List will be null if no objects are detected
                    Iterator<Recognition> updatedRecognitionsItr = updatedRecognitions.iterator();
                    telemetry.addData("Object Detected", updatedRecognitions.size()); // Tell the phones the number of detected objects
                    telemetry.update();
                    while(updatedRecognitionsItr.hasNext()) {
                        telemetry.addData("Status", "Filtering out double-detections....");
                        telemetry.update();
                        Recognition recognition = updatedRecognitionsItr.next();
                        if(updatedRecognitions.size() > 2) {
                            for(Recognition recognitionNested : updatedRecognitions) {
                                if((recognitionNested.getTop() + 10 > recognition.getTop()) && (recognitionNested.getTop() - 10 < recognition.getTop()) && (recognitionNested.getLeft() + 10 > recognition.getLeft() && recognitionNested.getLeft() - 10 < recognition.getLeft())) {
                                    if(recognitionNested != recognition) {
                                        remove = true;
                                    }
                                }
                            }
                            if(remove) {
                                updatedRecognitionsItr.remove();
                                remove = false;
                            }
                            if(updatedRecognitions.size() > 2) {
                                telemetry.addData("Status", "Filtering out crater...");
                                telemetry.update();
                                Recognition min1 = null;
                                Recognition min2 = null;
                                double minRecY = Double.MAX_VALUE;
                                for(Recognition minFind : updatedRecognitions) {
                                    if(minFind.getTop() < minRecY) {
                                        minRecY = minFind.getTop();
                                        min1 = minFind;
                                    }
                                }
                                minRecY = Double.MAX_VALUE;
                                for(Recognition minFind : updatedRecognitions) {
                                    if(minFind.getTop() < minRecY && minFind != min1) {
                                        minRecY = minFind.getTop();
                                        min2 = minFind;
                                    }
                                }
                                sampleMinerals[0] = min1;
                                sampleMinerals[1] = min2;
                                updatedRecognitionsItr = updatedRecognitions.iterator();
                                while(updatedRecognitionsItr.hasNext()) {
                                    recognition = updatedRecognitionsItr.next();
                                    if(recognition != min1 && recognition != min2) {
                                        updatedRecognitionsItr.remove();
                                    }
                                }
                            }
                        }
                    }
                    if(updatedRecognitions.size() == 3) { // If there are three detected objects (the minerals)
                        double goldMineralX = -987654; // Sets all of the object x positions to number well outside actual x range; if they still equal this number, they couldn't have been changed later in the code
                        double silverMineral1X = -987654;
                        double silverMineral2X = -987654;
                        for(Recognition recognition : updatedRecognitions) { // For each item in the list of recognized items
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) { // If the item is gold....
                                goldMineralX = (int)(recognition.getLeft()); // Set the gold x position to its x position
                            }
                            else if (silverMineral1X == -987654) { // If the item is silver and no silver has been assigned an x position yet....
                                silverMineral1X = (int)(recognition.getLeft()); // Set the first silver x position to its x position
                            }
                            else { // If the item is silver and another silver has been found
                                silverMineral2X = (int)(recognition.getLeft()); // Set the second silver x position to its x position
                            }
                        }
                        if (goldMineralX != -987654 && silverMineral1X != -987654 && silverMineral2X != -987654) { // If all of the minerals have new x positions....
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) { // If gold has the lowest x position
                                telemetry.addData("Gold Mineral Position?", "Left, x pos " + goldMineralX); // Tell phones it might be on the left
                                goldPos = -1;
                            }
                            else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) { // If gold has the highest x position
                                telemetry.addData("Gold Mineral Position?", "Right, x pos " + goldMineralX); // Tell phones it might be on the right
                                telemetry.update();
                                goldPos = 1;
                            }
                            else { // Otherwise....
                                telemetry.addData("Gold Mineral Position?", "Center, x pos " + goldMineralX);
                                telemetry.update();
                                goldPos = 0;
                            }
                            telemetry.update();
                        }
                    }
                    else if (updatedRecognitions.size() == 2) { // If only left two are visible
                        double goldMineralX = -987654; // Sets all of the object x positions to number well outside actual x range; if they still equal this number, they couldn't have been changed later in the code
                        double silverMineral1X = -987654;
                        double silverMineral2X = -987654;
                        for (Recognition recognition : updatedRecognitions) { // For each item in the list of recognized items
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) { // If the item is gold....
                                goldMineralX = (int)(recognition.getLeft()); // Set the gold x position to its x position (note that the phone's orientation reverses axes)
                            } else if (silverMineral1X == -987654) { // If the item is silver and no silver has been assigned an x position yet....
                                silverMineral1X = (int)(recognition.getLeft()); // Set the first silver x position to its x position
                            } else {
                                silverMineral2X = (int)(recognition.getLeft());
                            }
                            if (silverMineral2X == -987654 && goldMineralX != -987654 && silverMineral1X != -987654) {
                                if(goldMineralX > silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Center, x pos " + goldMineralX + "; silver x pos" + silverMineral1X);
                                    telemetry.update();
                                    goldPos = 0;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Left, x pos " + goldMineralX + "; silver x pos" + silverMineral1X);
                                    telemetry.update();
                                    goldPos = -1;
                                }
                            } else if (silverMineral2X != -987654) {
                                telemetry.addData("Gold Mineral Position", "Right, x pos " + goldMineralX);
                                telemetry.update();
                                goldPos = 1;
                            }
                        }
                    }
                    else {
                        try {
                            double goldMineralX = -987654; // Sets all of the object x positions to number well outside actual x range; if they still equal this number, they couldn't have been changed later in the code
                            double silverMineral1X = -987654;
                            double silverMineral2X = -987654;
                            for (Recognition recognition : sampleMinerals) { // For each item in the list of recognized items
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) { // If the item is gold....
                                    goldMineralX = (int) (recognition.getLeft()); // Set the gold x position to its x position (note that the phone's orientation reverses axes)
                                } else if (silverMineral1X == -987654) { // If the item is silver and no silver has been assigned an x position yet....
                                    silverMineral1X = (int)(recognition.getLeft()); // Set the first silver x position to its x position
                                } else {
                                    silverMineral2X = (int)(recognition.getLeft());
                                }
                                if (silverMineral2X == -987654 && goldMineralX != -987654 && silverMineral1X != -987654) {
                                    if (goldMineralX > silverMineral1X) {
                                        telemetry.addData("Gold Mineral Position", "Center, x pos " + goldMineralX + "; silver x pos" + silverMineral1X);
                                        telemetry.update();
                                        goldPos = 0;
                                    } else {
                                        telemetry.addData("Gold Mineral Position", "Left, x pos " + goldMineralX + "; silver x pos" + silverMineral1X);
                                        telemetry.update();
                                        goldPos = -1;
                                    }
                                } else if (silverMineral2X != -987654) {
                                    telemetry.addData("Gold Mineral Position", "Right, x pos " + goldMineralX);
                                    telemetry.update();
                                    goldPos = 1;
                                }
                            }
                        }
                        catch(Exception p_exception) {
                            telemetry.addData("Error", "The crater is in the frame and could not be filtered.  Please adjust the camera accordingly");
                            telemetry.update();
                        }
                    }
                    telemetry.update();
                    while (!(telemetry.update())) {}
                }
                else {
                    if(startTime > elapsedTime.time() + 10) {
                        telemetry.addData("Error: ", "No objects could be found.  Please consider adjusting the camera view on the field, unless this is " +
                                "a competition and it's too late, in which case your season just ended.");
                        telemetry.update();
                        break;
                    }
                }
            }
            else {
                telemetry.addData("Error: ", "The detector could not be initialized.  Please consider upgrading your phones and/or your programmer.");
                telemetry.update();
            }
        }
        return goldPos;
    }

    private void sample() {
        if(robot.intake.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            robot.intake.setPower(0);
            robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        robot.flipBox(0.61);
        sleep(500);
        robot.setInPower(1);
        sleep(1000);
        if(!(goldPos == 0 && elapsedTime.time() > 15)) {
            robot.setInPower(0);
            robot.flipBox(0.456);
        }
    }

    private void park() {
        if(goldPos != 1) {
            runToPoint(-5.2, 0.5);
        }
        else {
            runBackToPoint(-5.2, 0.5);
            robot.teamFlag.setPosition(0.026);
            frontTurn(-getAngle() + 135 + 180, 5);
        }
        robot.extend.setTargetPosition(-1000);
        robot.extend.setPower(1);
        runToPoint(-5.2, 2.25, (float)0.5);
        while(opModeIsActive() && robot.extend.isBusy()) {}
    }

    private void doubleSample() {
        double value = goldPos == 1 ? 46 : (goldPos == -1 ? 8.75 : 19.3);
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(!(robot.getUSDistance() + 1 > value && robot.getUSDistance() - 1 > value)) {
            double error = (value - robot.getUSDistance()) / 20;
            if(Math.abs(error) < 0.1 && error != 0) {
                error = (error / Math.abs(error)) * 0.1;
            }
            robot.setDrivePower(error, error, error, error);
        }
        robot.setDrivePower(0, 0, 0, 0);
        resetEncoders();
    }

    private void runToPoint(double newX, double newY) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]);
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = -getAngle() - 135.0 + angle;
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance, Math.abs(turnDistance) / 60.0);
        }
        int distance = calculateMove(Math.abs(newX - point[0]), Math.abs(newY - point[1]));
        encoderMovePreciseTimed(distance, 1, Math.abs(distance / 1500.0));
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runToPoint(double newX, double newY, int deviation) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]);
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = -getAngle() - 135.0 + angle;
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance, Math.abs(turnDistance) / 60.0);
        }
        int distance = calculateMove(Math.abs(newX - point[0]), Math.abs(newY - point[1]), deviation);
        encoderMovePreciseTimed(distance, 1, Math.abs(distance) / 1500.0);
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runToPoint(double newX, double newY, double angleError) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]) + angleError;
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = -getAngle() - 135.0 + angle;
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance, Math.abs(turnDistance) / 60.0);
        }
        int distance = calculateMove(Math.abs(newX - point[0]), Math.abs(newY - point[1]));
        encoderMovePreciseTimed(distance, 1, Math.abs(distance) / 1500.0);
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runToPoint(double newX, double newY, int deviation, double angleError) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]) + angleError;
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = -getAngle() - 135.0 + angle;
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance, Math.abs(turnDistance) / 60.0);
        }
        int distance = calculateMove(Math.abs(newX - point[0]), Math.abs(newY - point[1]), deviation);
        encoderMovePreciseTimed(distance, 1, Math.abs(distance) / 1500);
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runToPoint(double newX, double newY, float speed) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]);
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = -getAngle() - 135.0 + angle;
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance, Math.abs(turnDistance) / 60.0);
        }
        int distance = calculateMove(Math.abs(newX - point[0]), Math.abs(newY - point[1]));
        encoderMovePreciseTimed(distance, speed, Math.abs(distance / 1500.0));
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runBackToPoint(double newX, double newY) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]);
        }
        catch(Exception p_exception) {
            angle = -90;
        }
        double turnDistance = -getAngle() - 135.0 + angle + 180;
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance, Math.abs(turnDistance) / 60.0);
        }
        int distance = -calculateMove(Math.abs(newX - point[0]), Math.abs(newY - point[1]));
        encoderMovePreciseTimed(distance, 1, Math.abs(distance) / 1500.0);
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }


    private void encoderMovePreciseTimed(int rr, int rf, int lr, int lf, double speed, double timeLimit) { // Move encoders towards target position until the position is reached, or the time limit expires
        if (opModeIsActive() && robot.rrWheel != null && robot.rfWheel != null && robot.lrWheel != null && robot.lfWheel != null && robot.hangOne != null) {
            robot.rrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rrWheel.setTargetPosition(rr);
            robot.rfWheel.setTargetPosition(rf);
            robot.lfWheel.setTargetPosition(lf);
            robot.lrWheel.setTargetPosition(lr);
            try {
                robot.setDrivePower(-(lr / Math.abs(lr)) * speed, -(lf / Math.abs(lf)) * speed, -(rr / Math.abs(rr)) * speed, -(rf / Math.abs(rf)) * speed);
            }
            catch(Exception p_exception) {
                robot.setDrivePower(speed, speed, speed, speed);
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
            sleep(100);
        }
    }

    private void encoderMovePreciseTimed(int pos, double speed, double timeLimit) { // Move encoders towards target position until the position is reached, or the time limit expires
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rrWheel.setTargetPosition(pos);
        robot.rfWheel.setTargetPosition(pos);
        robot.lfWheel.setTargetPosition(pos);
        robot.lrWheel.setTargetPosition(pos);
        ElapsedTime limitTest = new ElapsedTime();
        try {
            robot.setDrivePower(-(pos / Math.abs(pos)) * Math.abs(speed), -(pos / Math.abs(pos)) * Math.abs(speed), -(pos / Math.abs(pos)) * Math.abs(speed), -(pos / Math.abs(pos)) * Math.abs(speed));
        }
        catch(Exception p_exception) {
            robot.setDrivePower(0, 0, 0, 0);
        }
        double maxSpeed = speed;
        speed = Math.min(0.1, Math.abs(maxSpeed));
        while((robot.rrWheel.isBusy() || robot.rfWheel.isBusy() || robot.lfWheel.isBusy() || robot.lrWheel.isBusy()) && opModeIsActive() && limitTest.time() < timeLimit) {
            double error = Math.abs(robot.rrWheel.getCurrentPosition() - robot.rrWheel.getTargetPosition()) / 500;
            if(!(speed >= maxSpeed) && !(error <= speed)) {
                speed += 0.1;
            }
            else {
                speed = Math.min(error, Math.abs(maxSpeed));
            }
            updateTelemetry();
            try {
                robot.setDrivePower(-(pos / Math.abs(pos)) * speed, -(pos / Math.abs(pos)) * speed, -(pos / Math.abs(pos)) * speed, -(pos / Math.abs(pos)) * speed);
            }
            catch(Exception p_exception) {
                robot.setDrivePower(0, 0, 0, 0);
            }
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

    private int calculateMove(double xDiff, double yDiff) {
        return((-((int)((12 * Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2)) - 9) * ticksPerInch))));
    }

    private int calculateMove(double xDiff, double yDiff, int deviation) {
        return((-((int)((12 * Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2)) - deviation) * ticksPerInch))));
    }

    /* Reset encoders and set modes to "Run to position" */
    private void resetEncoders() { // Reset encoder values and set encoders to "run to position" mode
        robot.rrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void turn(double angle, double time) {
        if(Math.abs(angle) < 0.1 || Math.abs(angle) + 0.1 % 360 < 0.2) { // Detects if turn is too insignificant
            return;
        }
        angle += (angleOffset * Math.abs(angle) / angle);
        double oldAngle;
        double angleIntended;
        double robotAngle;
        double lastError;
        double error = 0;
        ElapsedTime turnTime = new ElapsedTime();
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotAngle = getAngle();
        oldAngle = robotAngle;
        angleIntended = robotAngle + angle;
        if (angleIntended < robotAngle) { // Left turn
            if(angleIntended > 180) {
                angleIntended -= 360;
            }
            else if(angleIntended < -180) {
                angleIntended += 360;
            }
            while(opModeIsActive() && !(angleIntended - angleOffset < robotAngle && angleIntended + angleOffset > robotAngle) && turnTime.time() < time) {
                if(oldAngle > 0 || (Math.abs(angleIntended) == angleIntended && Math.abs(robotAngle) == robotAngle) || (Math.abs(angleIntended) != angleIntended && Math.abs(robotAngle) != robotAngle)) {
                    lastError = error;
                    error = Math.abs(robotAngle - angleIntended);
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                else {
                    lastError = error;
                    error = Math.abs(robotAngle - (angleIntended + (360 * -(Math.abs(angleIntended) / angleIntended))));
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                robot.setDrivePower(Math.min(-0.0075 * error, -0.1), Math.min(-0.0075 * error, -0.1), Math.max(0.0075 * error, 0.1), Math.max(0.0075 * error, 0.1));
                robotAngle = getAngle();
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
            while(opModeIsActive() && !(angleIntended - angleOffset < robotAngle && angleIntended + angleOffset > robotAngle) && turnTime.time() < time) {
                if(oldAngle < 0 || (Math.abs(angleIntended) == angleIntended && Math.abs(robotAngle) == robotAngle) || (Math.abs(angleIntended) != angleIntended && Math.abs(robotAngle) != robotAngle)) {
                    error = Math.abs(robotAngle - angleIntended);
                    if(error > 180) {
                        lastError = error;
                        error = Math.abs(robotAngle + angleIntended);
                        if(lastError != 0 && error > lastError) {
                            error = lastError;
                        }
                    }
                }
                else {
                    lastError = error;
                    error = Math.abs(robotAngle - (angleIntended + (360 * -(Math.abs(angleIntended) / angleIntended))));
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                robot.setDrivePower(Math.max(0.0075 * error, 0.1), Math.max(0.0075 * error, 0.1), Math.min(-0.0075 * error, -0.1), Math.min(-0.0075 * error, -0.1));
                robotAngle = getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        resetEncoders();
        /*
        error = -getAngle() + angleIntended;
        if(error > 180) {
            error -= 180;
        }
        else if(error < -180) {
            error += 180;
        }
        if(Math.abs(error) > 0.1 && time - turnTime.time() > 0.75 && turns <= 1) {
            turn(error, time - turnTime.time());
        }
        else {
            turns = 0;
        }
        */
    }


    private void frontTurn(double angle, double time) {
        if(Math.abs(angle) < 0.1 || Math.abs(angle) + 0.1 % 360 < 0.2) { // Detects if turn is too insignificant
            return;
        }
        angle += (angleOffset * Math.abs(angle) / angle);
        double oldAngle;
        double angleIntended;
        double robotAngle;
        double lastError;
        double error = 0;
        ElapsedTime turnTime = new ElapsedTime();
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotAngle = getAngle();
        oldAngle = robotAngle;
        angleIntended = robotAngle + angle;
        if (angleIntended < robotAngle) { // Left turn
            if(angleIntended > 180) {
                angleIntended -= 360;
            }
            else if(angleIntended < -180) {
                angleIntended += 360;
            }
            while(opModeIsActive() && !(angleIntended - angleOffset < robotAngle && angleIntended + angleOffset > robotAngle) && turnTime.time() < time) {
                if(oldAngle > 0 || (Math.abs(angleIntended) == angleIntended && Math.abs(robotAngle) == robotAngle) || (Math.abs(angleIntended) != angleIntended && Math.abs(robotAngle) != robotAngle)) {
                    lastError = error;
                    error = Math.abs(robotAngle - angleIntended);
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                else {
                    lastError = error;
                    error = Math.abs(robotAngle - (angleIntended + (360 * -(Math.abs(angleIntended) / angleIntended))));
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                robot.setDrivePower(0, Math.min(-0.0075 * error, -0.1), 0, Math.max(0.0075 * error, 0.1));
                robotAngle = getAngle();
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
            while(opModeIsActive() && !(angleIntended - angleOffset < robotAngle && angleIntended + angleOffset > robotAngle) && turnTime.time() < time) {
                if(oldAngle < 0 || (Math.abs(angleIntended) == angleIntended && Math.abs(robotAngle) == robotAngle) || (Math.abs(angleIntended) != angleIntended && Math.abs(robotAngle) != robotAngle)) {
                    error = Math.abs(robotAngle - angleIntended);
                    if(error > 180) {
                        lastError = error;
                        error = Math.abs(robotAngle + angleIntended);
                        if(lastError != 0 && error > lastError) {
                            error = lastError;
                        }
                    }
                }
                else {
                    lastError = error;
                    error = Math.abs(robotAngle - (angleIntended + (360 * -(Math.abs(angleIntended) / angleIntended))));
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                robot.setDrivePower(0, Math.max(0.0075 * error, 0.1), 0, Math.min(-0.0075 * error, -0.1));
                robotAngle = getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        resetEncoders();
        /*
        error = -getAngle() + angleIntended;
        if(error > 180) {
            error -= 180;
        }
        else if(error < -180) {
            error += 180;
        }
        if(Math.abs(error) > 0.1 && time - turnTime.time() > 0.75 && turns <= 1) {
            turn(error, time - turnTime.time());
        }
        else {
            turns = 0;
        }
        */
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

    /* Update telemetry with autonomous encoder positions */
    private void updateTelemetry() { // Update telemetry on autonomous statuses
        String telemetryString = "";
        telemetryString += "Remaining Distances\n";
        telemetryString += "  rr: " + (robot.rrWheel.getTargetPosition() - robot.rrWheel.getCurrentPosition());
        telemetryString += "  rf: " + (robot.rfWheel.getTargetPosition() - robot.rfWheel.getCurrentPosition());
        telemetryString += "  lr: " + (robot.lrWheel.getTargetPosition() - robot.lrWheel.getCurrentPosition());
        telemetryString += "  lf: " + (robot.lfWheel.getTargetPosition() - robot.lfWheel.getCurrentPosition());
        telemetryString += ("Robot angle: " + (getAngle() - 45));
        telemetry.addData("", telemetryString);
        telemetry.update();
    }


}
