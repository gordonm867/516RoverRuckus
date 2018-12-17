package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Iterator;
import java.util.List;

@Autonomous(name="GOFAutoCrater", group="GOF")
public class GOFAutonomousCrater extends LinearOpMode {

    /* Declare OpMode members */
    private                 GOFHardware         robot                   = GOFHardware.getInstance(); // Use the GOFHardware class
    private                 ElapsedTime         elapsedTime             = new ElapsedTime(); // Measure timing

    private static final    String              TFOD_MODEL_ASSET        = "RoverRuckus.tflite";
    private static final    String              LABEL_GOLD_MINERAL      = "Gold Mineral";
    private static final    String              LABEL_SILVER_MINERAL    = "Silver Mineral";
    private static final    String              VUFORIA_KEY             = "AWVhzQD/////AAABmWz790KTAURpmjOzox2azmML6FgjPO5DBf5SHQLIKvCsslmH9wp8b5zkCGfES8tt+8xslwaK7sd2h5H1jwmix26x+Eg5j60l00SlNiJMDAp5IOMWvhdJGZ8jJ8wFHCNkwERQG57JnrOXVSFDlc1sfum3oH68fEd8RrA570Y+WQda1fP8hYdZtbgG+ZDVG+9XyoDrToYU3FYl3WM1iUphAbHJz1BMFFnWJdbZzOicvqah/RwXqtxRDNlem3JdT4W95kCY5bckg92oaFIBk9n01Gzg8w5mFTReYMVI3Fne72/KpPRPJwblO0W9OI3o7djg+iPjxkKOeHUWW+tmi6r3LRaKTrIUfLfazRu0QwLA8Bgw";

    private                 GOFVuforiaLocalizer vuforia;
    private                 TFObjectDetector    detector;

    private                 boolean             remove;
    private                 boolean             doubleSample            = true;
    private                 boolean             yPressed                = false;
    private                 double              angleOffset             = 3;
    private                 double              kickOutPos              = 0.35;
    private                 double              kickReadyPos            = 0.2;
    private                 double              startTime               = elapsedTime.time();
    private                 double              ticksPerInch            = 560 / (4 * Math.PI);
    private                 double              turns                   = 0;
    private                 int                 goldPos                 = -2;

    @Override
    public void runOpMode() {
        /* Initialize hardware class */
        robot.init(hardwareMap);

        /* Reset encoders */
        if (robot.rrWheel != null && robot.rfWheel != null && robot.lfWheel != null && robot.lrWheel != null) {
            robot.teamFlag.setPosition(0.4);
            robot.rrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.hangOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        vuforiaInit(); // Initialize Vuforia
        detectInit(); // Initialize TensorFlwo

        GOFAutoTransitioner.transitionOnStop(this, "GOFTeleOp"); // Start TeleOp after autonomous ends

        while(!gamepad1.x) {
            telemetry.addData("Double Sampling is", (doubleSample ? "ON" : "OFF") + " - Press \"Y\" to change and \"X\" to finalize (on gamepad1)");
            telemetry.update();
            if(gamepad1.y && !yPressed) {
                doubleSample = !doubleSample;
                yPressed = true;
            }
            else {
                yPressed = false;
            }
        }

        telemetry.addData("Status: ", "Entering loop");
        telemetry.update();

        if(!isStopRequested()) {
            goldPos = detectGold();
        }

        telemetry.addData("Status", "Initialized with gold position " + goldPos);
        telemetry.update();

        waitForStart(); // Wait for user to press "PLAY"

        elapsedTime.reset();
        detector.shutdown();
        //vuforia.close();

        /*
        robot.playSound(goldPos);
        if (robot.soundError) {
            telemetry.addData("Error: ", "Unable to play sound.");
        }
        */

        /* Descend */
        descend();
        encoderMovePreciseTimed(258, -392, -422, 358, 1, 1); // side to side
        robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangOne.setTargetPosition(8050);
        robot.setHangPower(-1);
        turn(-getAngle(), 1);
        encoderMovePreciseTimed(-711, -86, -333, -815, 0.5, 1);
        if (opModeIsActive()) {
            robot.setKickPower(kickReadyPos); // Move kicker out of the way
        }
        telemetry.addData("Status", "Turning " + -getAngle());
        telemetry.update();
        turn(-getAngle(), 1);

        /* Move to gold */
        if (robot.rrWheel != null && robot.rfWheel != null && robot.lfWheel != null && robot.lrWheel != null && opModeIsActive()) {
            goldPos = Range.clip(goldPos, -2, 1);
            if (goldPos == -1) {
                leftCraterAuto();
            }
            else if (goldPos == 1) {
                rightCraterAuto();
            }
            else if (goldPos == 0 || goldPos == -2) {
                centerCraterAuto();
            }
        }
        else {
            telemetry.addData("Hardware problem detected", "A wheel is null, and I'm blaming hardware. Please fix it.");
            telemetry.update();
        }

        if(opModeIsActive()) {
            try {
                storeAngle();
            } catch (Exception p_exception) {
                telemetry.addData("Note", "Angle could not be saved; please manually initialize gyro in TeleOp");
                telemetry.update();
            }
        }
    }

    private void centerCraterAuto() {
        turn(-getAngle(), 1);
        // while(!gamepad1.a && opModeIsActive()) {}
        while(opModeIsActive() && !robot.bottomSensor.isPressed() && robot.hangOne.isBusy()) {
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        robot.setInPower(1);
        encoderMovePreciseTimed(calculateMove(1.049, 1.049, 7), 1, 0.75); // -784
        // while(!gamepad1.a && opModeIsActive()) {}
        robot.setInPower(0);
        encoderMovePreciseTimed(-calculateMove(1.049, 1.049, 7), 1, 0.75); // 784
        turn(83.14375, 3);
        encoderMovePreciseTimed(calculateMove(3, 3), 1, 1.5); // -1867
        // while(!gamepad1.a && opModeIsActive()) {}
        turn(45, 2);
        // while(!gamepad1.a && opModeIsActive()) {}
        encoderMovePreciseTimed(calculateMove(0, 3.5, 0), 1, 1.65); // -2139
        robot.teamFlag.setPosition(0.98);
        sleep(100);
        encoderMovePreciseTimed(calculateMove(0, 0.5, 0), 1, 0.35); // -2139
        // while(!gamepad1.a && opModeIsActive()) {}
        if(doubleSample) {
            turn(136.99875 - 8.375, 3);
            // while(!gamepad1.a && opModeIsActive()) {}
            robot.setInPower(1);
            encoderMovePreciseTimed(calculateMove(2.466, 2.466), 1, 1); // -1463
            robot.setInPower(0);
            // while(!gamepad1.a && opModeIsActive()) {}
            encoderMovePreciseTimed(-calculateMove(2.466, 2.466), 1, 1); // 1463
            // while(!gamepad1.a && opModeIsActive()){}
            turn(57.5, 2);
            // while(!gamepad1.a && opModeIsActive()) {}
            encoderMovePreciseTimed(calculateMove(0, 7.216), 1, 4); // -3457
        }
        else {
            encoderMovePreciseTimed(-calculateMove(0, 7.216), 1, 4); // -3457
        }
    }

    private void rightCraterAuto() {
        // while(!gamepad1.a && opModeIsActive()) {}
        turn(-(getAngle() - 42) + atan(-25) + 4, 3);
        while(opModeIsActive() && !robot.bottomSensor.isPressed() && robot.hangOne.isBusy()) {
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        robot.hangOne.setTargetPosition(robot.hangOne.getCurrentPosition());
        robot.setHangPower(0);
        // while(!gamepad1.a && opModeIsActive()) {}
        robot.setInPower(1);
        encoderMovePreciseTimed(calculateMove(0.4, 2), 1, 1); // -1338
        // while(!gamepad1.a && opModeIsActive()) {}
        robot.setInPower(0);
        encoderMovePreciseTimed(-calculateMove(0.4, 2), 1, 1); // 1338
        // while(!gamepad1.a && opModeIsActive()) {}
        turn(atan(1) - atan(-25) - 14, 3); // -136.65457618639903
        // while(!gamepad1.a && opModeIsActive()) {}
        encoderMovePreciseTimed(calculateMove(2, 2), 1, 1.5); // -1759
        // while(!gamepad1.a && opModeIsActive()) {}
        turn(45 + 3, 3);
        // while(!gamepad1.a && opModeIsActive()) {}
        robot.setInPower(1);
        encoderMovePreciseTimed(calculateMove(0, 4.5), 1, 3); // -4414
        robot.setInPower(0);
        robot.teamFlag.setPosition(0.98);
        sleep(100);
        // while(!gamepad1.a && opModeIsActive()) {}
        turn(atan(-7) + 94, 1.5); // -16.238841629692274
        // while(!gamepad1.a && opModeIsActive()) {}
        encoderMovePreciseTimed(-calculateMove(0.269, 7.24, 20), 1, 4.5); // 6946
    }

    private void leftCraterAuto() {
        turn(45.0 + atan(-1.0/15.0), 3); // 41.18592516570965
        // while(!gamepad1.a && opModeIsActive()) {}
        while(opModeIsActive() && !robot.bottomSensor.isPressed() && robot.hangOne.isBusy()) {
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        robot.hangOne.setPower(0);
        robot.hangOne.setTargetPosition(robot.hangOne.getCurrentPosition());
        robot.setInPower(1);
        encoderMovePreciseTimed(calculateMove(0.133, 2), 1, 0.75); // -670
        // while(!gamepad1.a && opModeIsActive()) {}
        robot.setInPower(0);
        encoderMovePreciseTimed(-calculateMove(0.133, 2), 1, 0.75); // 670
        // while(!gamepad1.a && opModeIsActive()) {}
        turn(-getAngle() + 83.14375, 3); // 48.81407483429035
        // while(!gamepad1.a && opModeIsActive()) {}
        encoderMovePreciseTimed(calculateMove(3, 3), 1, 1.5); // -1867
        // while(!gamepad1.a && opModeIsActive()) {}
        turn(45, 2);
        // while(!gamepad1.a && opModeIsActive()) {}
        encoderMovePreciseTimed(calculateMove(0, 3, 0), 1, 1.5); // Part of -2139
        robot.teamFlag.setPosition(0.98);
        sleep(100);
        encoderMovePreciseTimed(calculateMove(0, 1, 0), 1, 0.5); // Part of -2139
        // while(!gamepad1.a && opModeIsActive()) {}
        if(doubleSample) {
            turn(90 + atan(1.0/4.0) - 8, 3); // 104.03624346792648
            // while(!gamepad1.a && opModeIsActive()) {}
            robot.setInPower(1);
            encoderMovePreciseTimed(calculateMove(4, 1), 1, 1.5); // -1803
            robot.setInPower(0);
            // while(!gamepad1.a && opModeIsActive()) {}
            turn(-atan(1.0/4.0) + atan(-1.0/2.0) + 33.3125, 3); // -40.60129464500447
            // while(!gamepad1.a && opModeIsActive()) {}
            encoderMovePreciseTimed(calculateMove(2, 3.8), 1, 0.75); // -461
        }
        else {
            encoderMovePreciseTimed(-calculateMove(0, 6.9), 1, 3); // 4259
        }
    }

    private double atan(double num) { // Returns atan in degrees
        return(Math.atan(num) * (180 / Math.PI));
    }

    private void descend() {
        resetEncoders();
        robot.hangOne.setTargetPosition(-8058);
        while (opModeIsActive() && robot.hangOne.getCurrentPosition() > -8058) {
            telemetry.addData("h1: ", "" + robot.hangOne.getCurrentPosition());
            telemetry.addData("Current angle", (getAngle() - 45));
            telemetry.update();
            robot.setHangPower(-1);
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
                List<Recognition> updatedRecognitions = detector.getUpdatedRecognitions(); // ArrayList of detected objects
                if (updatedRecognitions != null) { // List will be null if no objects are detected
                    Iterator<Recognition> updatedRecognitionsItr = updatedRecognitions.iterator();
                    telemetry.addData("Object Detected", updatedRecognitions.size()); // Tell the phones the number of detected objects
                    telemetry.update();
                    while (updatedRecognitionsItr.hasNext()) {
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
                            if (remove) {
                                updatedRecognitionsItr.remove();
                                remove = false;
                            }
                            if(updatedRecognitions.size() > 2) {
                                telemetry.addData("Status", "Filtering out crater....");
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
                                updatedRecognitionsItr = updatedRecognitions.iterator();
                                while (updatedRecognitionsItr.hasNext()) {
                                    recognition = updatedRecognitionsItr.next();
                                    if(recognition != min1 && recognition != min2) {
                                        updatedRecognitionsItr.remove();
                                    }
                                }
                            }
                        }
                    }
                    if (updatedRecognitions.size() == 3) { // If there are three detected objects (the minerals)
                        double goldMineralX = -987654; // Sets all of the object x positions to number well outside actual x range; if they still equal this number, they couldn't have been changed later in the code
                        double silverMineral1X = -987654;
                        double silverMineral2X = -987654;
                        for (Recognition recognition : updatedRecognitions) { // For each item in the list of recognized items
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) { // If the item is gold....
                                goldMineralX = (int)(recognition.getLeft()); // Set the gold x position to its x position
                            } else if (silverMineral1X == -987654) { // If the item is silver and no silver has been assigned an x position yet....
                                silverMineral1X = (int)(recognition.getLeft()); // Set the first silver x position to its x position
                            } else { // If the item is silver and another silver has been found
                                silverMineral2X = (int)(recognition.getLeft()); // Set the second silver x position to its x position
                            }
                        }
                        if (goldMineralX != -987654 && silverMineral1X != -987654 && silverMineral2X != -987654) { // If all of the minerals have new x positions....
                            telemetry.addData("Gold Mineral Position?", "Left, x pos " + goldMineralX); // Tell phones it might be on the left
                            telemetry.update();
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) { // If gold has the lowest x position
                                telemetry.addData("Gold Mineral Position", "Left"); // Tell phones it's "definitely" on the left
                                goldPos = -1;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) { // If gold has the highest x position
                                telemetry.addData("Gold Mineral Position?", "Right, x pos " + goldMineralX); // Tell phones it might be on the right
                                telemetry.update();
                                goldPos = 1;
                            } else { // Otherwise....
                                telemetry.addData("Gold Mineral Position?", "Center, x pos " + goldMineralX);
                                telemetry.update();
                                goldPos = 0;
                            }
                            telemetry.update();
                        }
                    } else if (updatedRecognitions.size() == 2) { // If only left two are visible
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
                                if (goldMineralX > silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Right, x pos " + goldMineralX + "; silver x pos" + silverMineral1X);
                                    telemetry.update();
                                    goldPos = 1;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center, x pos " + goldMineralX + ";silver x pos" + silverMineral1X);
                                    telemetry.update();
                                    goldPos = 0;
                                }
                            } else if (silverMineral2X != -987654) {
                                telemetry.addData("Gold Mineral Position", "Left, x pos " + goldMineralX);
                                telemetry.update();
                                goldPos = -1;
                            }
                        }
                    }
                    telemetry.update();
                    while (!(telemetry.update())) {
                    }
                } else {
                    if (startTime > elapsedTime.time() + 10) {
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
            robot.setDrivePower(-(lr / Math.abs(lr)) * speed, -(lf / Math.abs(lf)) * speed, -(rr / Math.abs(rr)) * speed, -(rf / Math.abs(rf)) * speed);
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

    private void encoderMovePreciseTimed(int pos, double speed, double timeLimit) { // Move encoders towards target position until the position is reached, or the time limit expires
        if (opModeIsActive() && robot.rrWheel != null && robot.rfWheel != null && robot.lrWheel != null && robot.lfWheel != null && robot.hangOne != null) {
            robot.rrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rrWheel.setTargetPosition(pos);
            robot.rfWheel.setTargetPosition(pos);
            robot.lfWheel.setTargetPosition(pos);
            robot.lrWheel.setTargetPosition(pos);
            robot.setDrivePower(-(pos / Math.abs(pos)) * speed, -(pos / Math.abs(pos)) * speed, -(pos / Math.abs(pos)) * speed, -(pos / Math.abs(pos)) * speed);
            ElapsedTime limitTest = new ElapsedTime();
            // double startAngle = getAngle();
            while ((robot.rrWheel.isBusy() || robot.rfWheel.isBusy() || robot.lrWheel.isBusy() || robot.lfWheel.isBusy()) && opModeIsActive() && limitTest.time() < timeLimit) {
                updateTelemetry();
                /*
                pos = pos - robot.rrWheel.getCurrentPosition();
                if(Math.abs(startAngle - getAngle()) > 3) {
                    robot.setDrivePower(0, 0, 0, 0);
                    turn(startAngle - getAngle(), 0.25);
                    robot.rrWheel.setTargetPosition(pos);
                    robot.rfWheel.setTargetPosition(pos);
                    robot.lfWheel.setTargetPosition(pos);
                    robot.lrWheel.setTargetPosition(pos);
                } */
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
        robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void turn(double angle, double time) {
        turns += 1;
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

    private void vuforiaInit() { // Initialize Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        // parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Wc1"); // Use external camera
        vuforia = new GOFVuforiaLocalizer(parameters);
    }

    private void detectInit() { // Initialize TensorFlow detector
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        detector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        detector.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        if (!(detector == null)) {
            detector.activate(); // Begin detection
        }
    }

    /* Update telemetry with autonomous encoder positions */
    private void updateTelemetry() { // Update telemetry on autonomous statuses
        String telemetryString = "";
        telemetryString += "Remaining Distances\n";
        telemetryString += "  rr: " + (robot.rrWheel.getTargetPosition() - robot.rrWheel.getCurrentPosition());
        telemetryString += "  rf: " + (robot.rfWheel.getTargetPosition() - robot.rfWheel.getCurrentPosition());
        telemetryString += "  lr: " + (robot.lrWheel.getTargetPosition() - robot.lrWheel.getCurrentPosition());
        telemetryString += "  lf: " + (robot.lfWheel.getTargetPosition() - robot.lfWheel.getCurrentPosition());
        telemetryString += "  h1: " + (robot.hangOne.getTargetPosition() - robot.hangOne.getCurrentPosition());
        telemetryString += ("Robot angle: " + (getAngle() - 45));
        telemetry.addData("", telemetryString);
        telemetry.update();
    }

    private void storeAngle() throws IOException {
        double robotAngle = getAngle();
        File fileDir = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + File.separator);
        File file = new File(fileDir, "gof.txt");
        boolean created = file.exists();
        if (!created) {
            created = file.mkdirs();
        }
        if(created) {
            File txtfile = new File(file, "gof.txt");
            FileWriter writer = new FileWriter(txtfile);
            writer.append(Double.toString(robotAngle));
            writer.close();
        }
    }


    /*
         ==================================================

                D E P R E C A T E D  M E T H O D S

         ==================================================
    */

    /* ENCODER MOVEMENTS */

    // Encoder move methods have been replaced with timed method to prevent an over-consumption of time
    // as encoders attempt to perfect some movements.  Former methods have been deprecated.
/*
    @Deprecated
    public void encoderMove(int rr, int rf, int lr, int lf) {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rrWheel.setTargetPosition(rr);
        robot.rfWheel.setTargetPosition(rf);
        robot.lfWheel.setTargetPosition(lr);
        robot.lrWheel.setTargetPosition(lf);
        robot.setDrivePower(0.75, 0.75, 0.75, 0.75);
        while (robot.rrWheel.isBusy() || robot.rfWheel.isBusy() || robot.lrWheel.isBusy() || robot.lfWheel.isBusy()) {
            updateTelemetry();
        }
        resetEncoders();
    }

    @Deprecated
    public void encoderMovePrecise(int rr, int rf, int lr, int lf, double speed) {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rrWheel.setTargetPosition(rr);
        robot.rfWheel.setTargetPosition(rf);
        robot.lfWheel.setTargetPosition(lr);
        robot.lrWheel.setTargetPosition(lf);
        robot.setDrivePower(speed, speed, speed, speed);
        while ((robot.rrWheel.isBusy() || robot.rfWheel.isBusy() || robot.lrWheel.isBusy() || robot.lfWheel.isBusy()) && opModeIsActive()) {
            updateTelemetry();
        }
        resetEncoders();
    }

    /* KINEMATIC MOVEMENT EQUATIONS */

    // These equations used the equation x = (vix)(t) + (1/2)(ax)(t^2) for movement, assuming that initial
    // velocity was always zero.  However, due to the inaccuracy of the accelerometer, the slight delay
    // before the motors could be fully stopped after reaching the intended distance, and the difficulty
    // of obtaining precise field distances and angular measurements, encoder counts appear to be the
    // more feasible solution.  These methods have thus been deprecated.

    /*
    @Deprecated
    public void moveDistanceRight(double meters) {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime time = new ElapsedTime();
        double distance = 0;
        while (distance < meters && opModeIsActive()) {
            robot.setDrivePower(-0.25, 0.25, 0.25, -0.25);
            Acceleration g0accel = null;
            Acceleration g1accel = null;
            double robotAccel;
            if (robot.gyro0 != null) {
                g0accel = robot.gyro0.getGravity();
            }
            if (robot.gyro1 != null) {
                g1accel = robot.gyro1.getGravity();
            }
            if (g0accel != null && g1accel != null) {
                robotAccel = ((Math.sqrt(Math.pow(g0accel.xAccel, 2) + Math.pow(g0accel.yAccel, 2)) + Math.sqrt(Math.pow(g1accel.xAccel, 2) + Math.pow(g1accel.yAccel, 2))) / 2);
            } else if (g0accel != null) {
                robotAccel = Math.sqrt(Math.pow(g0accel.xAccel, 2) + Math.pow(g0accel.yAccel, 2));
            } else if (g1accel != null) {
                robotAccel = Math.sqrt(Math.pow(g1accel.xAccel, 2) + Math.pow(g1accel.yAccel, 2));
            } else {
                robotAccel = 0;
            }
            distance = ((0.5) * robotAccel * Math.pow(time.time(), 2));
            telemetry.addData("Distance Moved", "" + distance);
            telemetry.addData("Distance Remaining", "" + (meters - distance));
            telemetry.addData("Intended Total Distance", "" + meters);
            telemetry.update();
        }
        robot.setDrivePower(0, 0, 0, 0);
        resetEncoders();
    }

    @Deprecated
    public void moveDistanceLeft(double meters) {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime time = new ElapsedTime();
        double distance = 0;
        while (distance < meters && opModeIsActive()) {
            robot.setDrivePower(0.25, -0.25, -0.25, 0.25);
            Acceleration g0accel = null;
            Acceleration g1accel = null;
            double robotAccel;
            if (robot.gyro0 != null) {
                g0accel = robot.gyro0.getGravity();
            }
            if (robot.gyro1 != null) {
                g1accel = robot.gyro1.getGravity();
            }
            if (g0accel != null && g1accel != null) {
                robotAccel = ((g0accel.xAccel + g1accel.xAccel) / 2);
            } else if (g0accel != null) {
                robotAccel = g0accel.xAccel;
            } else if (g1accel != null) {
                robotAccel = g1accel.xAccel;
            } else {
                robotAccel = 0;
            }
            distance = ((0.5) * robotAccel * Math.pow(time.time(), 2));
            telemetry.addData("Distance Moved", "" + distance);
            telemetry.addData("Distance Remaining", "" + (meters - distance));
            telemetry.addData("Intended Total Distance", "" + meters);
            telemetry.update();
        }
        robot.setDrivePower(0, 0, 0, 0);
        resetEncoders();
    }

    @Deprecated
    public void moveDistanceStraight(double meters) {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime time = new ElapsedTime();
        double distance = 0;
        while (distance < meters && opModeIsActive()) {
            robot.setDrivePower(0.25, 0.25, 0.25, 0.25);
            Acceleration g0accel = null;
            Acceleration g1accel = null;
            double robotAccel;
            if (robot.gyro0 != null) {
                g0accel = robot.gyro0.getGravity();
            }
            if (robot.gyro1 != null) {
                g1accel = robot.gyro1.getGravity();
            }
            if (g0accel != null && g1accel != null) {
                robotAccel = ((Math.sqrt(Math.pow(g0accel.xAccel, 2) + Math.pow(g0accel.yAccel, 2)) + Math.sqrt(Math.pow(g1accel.xAccel, 2) + Math.pow(g1accel.yAccel, 2))) / 2);
            } else if (g0accel != null) {
                robotAccel = Math.sqrt(Math.pow(g0accel.xAccel, 2) + Math.pow(g0accel.yAccel, 2));
            } else if (g1accel != null) {
                robotAccel = Math.sqrt(Math.pow(g1accel.xAccel, 2) + Math.pow(g1accel.yAccel, 2));
            } else {
                robotAccel = 0;
            }
            distance = ((0.5) * robotAccel * Math.pow(time.time(), 2));
            telemetry.addData("Distance Moved", "" + distance);
            telemetry.addData("Distance Remaining", "" + (meters - distance));
            telemetry.addData("Intended Total Distance", "" + meters);
            telemetry.update();
        }
        robot.setDrivePower(0, 0, 0, 0);
        resetEncoders();
    }
/*
    @Deprecated
    public void moveDistanceBack(double meters) {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime time = new ElapsedTime();
        double distance = 0;
        while (distance > meters && opModeIsActive()) {
            robot.setDrivePower(-0.25, -0.25, -0.25, -0.25);
            Acceleration g0accel = null;
            Acceleration g1accel = null;
            double robotAccel;
            if (robot.gyro0 != null) {
                g0accel = robot.gyro0.getGravity();
            }
            if (robot.gyro1 != null) {
                g1accel = robot.gyro1.getGravity();
            }
            if (g0accel != null && g1accel != null) {
                robotAccel = ((Math.sqrt(Math.pow(g0accel.xAccel, 2) + Math.pow(g0accel.yAccel, 2)) + Math.sqrt(Math.pow(g1accel.xAccel, 2) + Math.pow(g1accel.yAccel, 2))) / 2);
            } else if (g0accel != null) {
                robotAccel = Math.sqrt(Math.pow(g0accel.xAccel, 2) + Math.pow(g0accel.yAccel, 2));
            } else if (g1accel != null) {
                robotAccel = Math.sqrt(Math.pow(g1accel.xAccel, 2) + Math.pow(g1accel.yAccel, 2));
            } else {
                robotAccel = 0;
            }
            distance = ((0.5) * robotAccel * Math.pow(time.time(), 2));
            telemetry.addData("Distance Moved", "" + distance);
            telemetry.addData("Distance Remaining", "" + (meters - distance));
            telemetry.addData("Intended Total Distance", "" + meters);
            telemetry.update();
        }
        robot.setDrivePower(0, 0, 0, 0);
        resetEncoders();
    }

    /* STRAIGHTEN ROBOT */

    // Due to the difficulty associated with accurately straightening the gyro, straightening methods
    // have been replaced with turn P control and experimental error correction.  The former methods,
    // which turn to the nearest multiple of the given number of degrees (which defaults to 45º using
    // method overloading) have been deprecated.
/*
    @Deprecated
    public void adjustAngle() {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        } else if (g0angles != null) {
            robotAngle = g0angles.firstAngle;
        } else if (g1angles != null) {
            robotAngle = g1angles.firstAngle;
        } else {
            robotAngle = 0;
        }
        while (opModeIsActive() && Math.abs(robotAngle) % 45 > angleOffset) {
            g0angles = null;
            g1angles = null;
            if (robot.gyro0 != null) {
                g0angles = robot.gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from first gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
            }
            if (robot.gyro1 != null) {
                g1angles = robot.gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from second gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
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
            if (robotAngle % 45 > 22.5) {
                robot.setDrivePower(0.2 * (45 - (robotAngle % 45)), 0.2 * (45 - (robotAngle % 45)), -0.2 * (45 - (robotAngle % 45)), -0.2 * (45 - (robotAngle % 45)));
                if (Math.abs(robotAngle) % 45 < angleOffset) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
            else {
                robot.setDrivePower(-0.2 * (robotAngle % 45), -0.2 * (robotAngle % 45), 0.2 * (robotAngle % 45), 0.2 * (robotAngle % 45));
                if (Math.abs(robotAngle) % 45 < angleOffset) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
        }
        resetEncoders();
    }

    @Deprecated
    public void adjustAngle(double time) {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        } else if (g0angles != null) {
            robotAngle = g0angles.firstAngle;
        } else if (g1angles != null) {
            robotAngle = g1angles.firstAngle;
        } else {
            robotAngle = 0;
        }
        ElapsedTime turnTime = new ElapsedTime();
        while (opModeIsActive() && turnTime.time() < time && Math.abs(robotAngle) % 45 > angleOffset) {
            g0angles = null;
            g1angles = null;
            if (robot.gyro0 != null) {
                g0angles = robot.gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from first gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
            }
            if (robot.gyro1 != null) {
                g1angles = robot.gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from second gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
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
            if (robotAngle % 45 > 22.5) {
                robot.setDrivePower(0.1 * (45 - (robotAngle % 45)), 0.1 * (45 - (robotAngle % 45)), -0.1 * (45 - (robotAngle % 45)), -0.1 * (45 - (robotAngle % 45)));
                g0angles = null;
                g1angles = null;
                if (robot.gyro0 != null) {
                    g0angles = robot.gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from first gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
                }
                if (robot.gyro1 != null) {
                    g1angles = robot.gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from second gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
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
                if (((Math.abs(robotAngle) % 45) < angleOffset) || turnTime.time() > time) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
            else {
                robot.setDrivePower(-0.1 * (robotAngle % 45), -0.1 * (robotAngle % 45), 0.1 * (robotAngle % 45), 0.1 * (robotAngle % 45));
                g0angles = null;
                g1angles = null;
                if (robot.gyro0 != null) {
                    g0angles = robot.gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from first gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
                }
                if (robot.gyro1 != null) {
                    g1angles = robot.gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from second gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
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
                if (((Math.abs(robotAngle) % 45) < angleOffset) || turnTime.time() > time) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
        }
        resetEncoders();
    }

    @Deprecated
    public void adjustAngle(double time, double straight) {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Orientation g0angles = null;
        Orientation g1angles = null;
        if(robot.gyro0 != null) {
            g0angles = robot.gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from first gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
        }
        if(robot.gyro1 != null) {
            g1angles = robot.gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from second gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
        }
        double robotAngle;
        if (g0angles != null && g1angles != null) {
            robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2); // Average angle measures to determine actual robot angle
        } else if (g0angles != null) {
            robotAngle = g0angles.firstAngle;
        } else if (g1angles != null) {
            robotAngle = g1angles.firstAngle;
        } else {
            robotAngle = 0;
        }
        ElapsedTime turnTime = new ElapsedTime();
        while (opModeIsActive() && turnTime.time() < time && Math.abs(robotAngle) % straight > angleOffset) {
            if(robot.gyro0 != null) {
                g0angles = robot.gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from first gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
            }
            if(robot.gyro1 != null) {
                g1angles = robot.gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from second gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
            }
            if (robotAngle % straight > straight / 2) {
                robot.setDrivePower(0.1 * (straight - (robotAngle % straight)), 0.1 * (straight - (robotAngle % straight)), -0.1 * (straight - (robotAngle % straight)), -0.1 * (straight - (robotAngle % straight)));
                if (g0angles != null && g1angles != null) {
                    robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2); // Average angle measures to determine actual robot angle
                } else if (g0angles != null) {
                    robotAngle = g0angles.firstAngle;
                } else if (g1angles != null) {
                    robotAngle = g1angles.firstAngle;
                } else {
                    robotAngle = 0;
                }
                if (((Math.abs(robotAngle) % straight) < angleOffset) || turnTime.time() > time) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
            else {
                robot.setDrivePower(-0.1 * (robotAngle % straight), -0.1 * (robotAngle % straight), 0.1 * (robotAngle % straight), 0.1 * (robotAngle % straight));
                if (g0angles != null && g1angles != null) {
                    robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2); // Average angle measures to determine actual robot angle
                } else if (g0angles != null) {
                    robotAngle = g0angles.firstAngle;
                } else if (g1angles != null) {
                    robotAngle = g1angles.firstAngle;
                } else {
                    robotAngle = 0;
                }
                if (((Math.abs(robotAngle) % straight) < angleOffset) || turnTime.time() > time) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
        }
        resetEncoders();
    }

    @Deprecated
    public void adjustAngle(double time, double straight, double precision) {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        } else if (g0angles != null) {
            robotAngle = g0angles.firstAngle;
        } else if (g1angles != null) {
            robotAngle = g1angles.firstAngle;
        } else {
            robotAngle = 0;
        }
        ElapsedTime turnTime = new ElapsedTime();
        while (opModeIsActive() && turnTime.time() < time && Math.abs(robotAngle) % 45 > precision) {
            if (robot.gyro0 != null) {
                g0angles = robot.gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from first gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
            }
            if (robot.gyro1 != null) {
                g1angles = robot.gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); // Get z axis angle from second gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
            }
            if (robotAngle % straight > straight / 2) {
                robot.setDrivePower(0.1 * (straight - (robotAngle % straight)), 0.1 * (straight - (robotAngle % straight)), -0.1 * (straight - (robotAngle % straight)), -0.1 * (straight - (robotAngle % straight)));
                if (g0angles != null && g1angles != null) {
                    robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2); // Average angle measures to determine actual robot angle
                } else if (g0angles != null) {
                    robotAngle = g0angles.firstAngle;
                } else if (g1angles != null) {
                    robotAngle = g1angles.firstAngle;
                } else {
                    robotAngle = 0;
                }
                if (((Math.abs(robotAngle) % straight) < precision) || turnTime.time() > time) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
            else {
                robot.setDrivePower(-0.1 * (robotAngle % straight), -0.1 * (robotAngle % straight), 0.1 * (robotAngle % straight), 0.1 * (robotAngle % straight));
                if (g0angles != null && g1angles != null) {
                    robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2); // Average angle measures to determine actual robot angle
                } else if (g0angles != null) {
                    robotAngle = g0angles.firstAngle;
                } else if (g1angles != null) {
                    robotAngle = g1angles.firstAngle;
                } else {
                    robotAngle = 0;
                }
                if (((Math.abs(robotAngle) % straight) < precision) || turnTime.time() > time) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
        }
        resetEncoders();
    }
    */
}
