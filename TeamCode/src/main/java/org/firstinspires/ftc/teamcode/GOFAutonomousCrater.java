package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Iterator;
import java.util.List;

@Autonomous(name="GOFAutoCrater", group="GOF")
public class GOFAutonomousCrater extends LinearOpMode implements Runnable {

    /* Declare OpMode members */
    private                 BNO055IMU           gyro0;
    private                 BNO055IMU           gyro1;
    private                 GOFHardware         robot                   = GOFHardware.getInstance(); // Use the GOFHardware class
    private                 ElapsedTime         elapsedTime             = new ElapsedTime(); // Measure timing
    private                 Thread              vuforiaThread;

    private static final    String              TFOD_MODEL_ASSET        = "RoverRuckus.tflite";
    private static final    String              LABEL_GOLD_MINERAL      = "Gold Mineral";
    private static final    String              LABEL_SILVER_MINERAL    = "Silver Mineral";
    private static final    String              VUFORIA_KEY             = "AWVhzQD/////AAABmWz790KTAURpmjOzox2azmML6FgjPO5DBf5SHQLIKvCsslmH9wp8b5zkCGfES8tt+8xslwaK7sd2h5H1jwmix26x+Eg5j60l00SlNiJMDAp5IOMWvhdJGZ8jJ8wFHCNkwERQG57JnrOXVSFDlc1sfum3oH68fEd8RrA570Y+WQda1fP8hYdZtbgG+ZDVG+9XyoDrToYU3FYl3WM1iUphAbHJz1BMFFnWJdbZzOicvqah/RwXqtxRDNlem3JdT4W95kCY5bckg92oaFIBk9n01Gzg8w5mFTReYMVI3Fne72/KpPRPJwblO0W9OI3o7djg+iPjxkKOeHUWW+tmi6r3LRaKTrIUfLfazRu0QwLA8Bgw";

    private                 GOFVuforiaLocalizer vuforia;
    private                 TFObjectDetector    detector;

    private                 boolean             remove;
    private                 double              angleOffset             = 0.25;
    private                 double              kickOutPos              = 0.35;
    private                 double              kickReadyPos            = 0.2;
    private                 double              startTime               = elapsedTime.time();
    private                 int                 goldPos                 = -2;

    @Override
    public void runOpMode() {
        /* Initialize hardware class */
        robot.init(hardwareMap);
        vuforiaThread = new Thread(new GOFAutonomousCrater());
        vuforiaThread.start();

        vuforiaInit();
        detectInit();

        GOFAutoTransitioner.transitionOnStop(this, "GOFTeleOp");

        while(!isStopRequested() && vuforiaThread.isAlive() && !vuforiaThread.isInterrupted()) {}

        telemetry.addData("Status: ", "Entering loop");
        telemetry.update();

        if(!isStopRequested()) {
            goldPos = detectGold();
        }

        telemetry.addData("Status", "Initialized with gold position " + goldPos);
        telemetry.update();

        waitForStart(); // Wait for user to press "PLAY"

        detector.shutdown();
        vuforia.close();

        robot.playSound(goldPos);
        if (robot.soundError) {
            telemetry.addData("Error: ", "Unable to play sound.");
        }

        /* Descend */
       descend();

        /* Move to gold */
        if (robot.rrWheel != null && robot.rfWheel != null && robot.lfWheel != null && robot.lrWheel != null && opModeIsActive()) {
            if (goldPos == -1) {
                leftCraterAuto();
            } else if (goldPos == 1) {
                rightCraterAuto();
            } else if (goldPos == 0 || goldPos == -2) {
                centerCraterAuto();
            }
        }

        try {
            storeAngle();
        }
        catch(Exception p_exception) {
            telemetry.addData("Note", "Angle could not be saved; please manually initialize gyro in TeleOp");
            telemetry.update();
        }
    }

    private void centerCraterAuto() {
        encoderMovePreciseTimed(235, -278, -264, 285, 1, 1); // side to side
        encoderMovePreciseTimed(-525, -542, -516, -534, 0.5, 1);
        robot.hangOne.setTargetPosition(8263);
        robot.setHangPower(-1);
        if (opModeIsActive()) {
            robot.setKickPower(kickReadyPos); // Move kicker out of the way
        }
        encoderMovePreciseTimed(43, 65, -41, -54, 0.8, 2);
        while(opModeIsActive() && !robot.bottomSensor.isPressed() && robot.hangOne.isBusy()) {
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        robot.setInPower(0.5);
        encoderMovePreciseTimed(-458, -510, -502, -539, 0.8, 2);
        encoderMovePreciseTimed(-176, -205, -185, -216, 0.8, 2);
    }

    private void rightCraterAuto() {
        encoderMovePreciseTimed(235, -278, -264, 285, 1, 1); // side to side
        encoderMovePreciseTimed(-525, -542, -516, -534, 0.5, 1);
        robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangOne.setTargetPosition(8263);
        robot.setHangPower(-1);
        if (opModeIsActive()) {
            robot.setKickPower(kickReadyPos); // Move kicker out of the way
        }
        encoderMovePreciseTimed(-979, 1009, 928, -961, 0.8, 2);
        while(opModeIsActive() && !robot.bottomSensor.isPressed() && robot.hangOne.isBusy()) {
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        robot.setHangPower(0);
        robot.setInPower(0.5);
        encoderMovePreciseTimed(-546, -609, -491, -595, 0.5, 2);
        adjustAngle(0.5);
        robot.setInPower(0);
        encoderMovePreciseTimed(514, 503, 529, 536, 1, 1);
        encoderMovePreciseTimed(-849, -1035, 873, 933, 1, 1);
        adjustAngle(0.5);
        encoderMovePreciseTimed(-1982, -2015, -1971, -1967, 1, 1.5);
        adjustAngle(0.5);
        encoderMovePreciseTimed(-159, -164, 112, 204, 1, 2);
        encoderMovePreciseTimed(-926, -951, -614, -564, 1, 1);
        robot.setInPower(0.5);
        encoderMovePreciseTimed(-1519, -1519, -1566, -1566, 1, 2);
        robot.setInPower(0);
        // DROP HERE
        encoderMovePreciseTimed(731, 672, 690, 695, -1, 2);
        encoderMovePreciseTimed(-494, 1951, 1753, -418, 1, 2);
        encoderMovePreciseTimed(711, 698, 706, 696, -1, 2);
        robot.hangOne.setTargetPosition(-8263);
        robot.setHangPower(1);
    }

    private void leftCraterAuto() {
        encoderMovePreciseTimed(235, -278, -264, 285, 1, 1); // side to side
        encoderMovePreciseTimed(-525, -542, -516, -534, 0.5, 1);
        robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangOne.setTargetPosition(8263);
        robot.setHangPower(-1);
        if (opModeIsActive()) {
            robot.setKickPower(kickReadyPos); // Move kicker out of the way
        }
        encoderMovePreciseTimed(555, -780, -674, 877, 0.8, 2);
        while(opModeIsActive() && !robot.bottomSensor.isPressed() && robot.hangOne.isBusy()) {
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        robot.setInPower(0.5);
        encoderMovePreciseTimed(34, 61, -58, -79, 0.8, 1);
        encoderMovePreciseTimed(-512, -591, -525, -601, 0.8, 2);
        robot.setInPower(0);
        encoderMovePreciseTimed(-139, -179, -138, -151, 0.8, 2);
    }

    private void descend() {
        resetEncoders();
        robot.hangOne.setTargetPosition(-8263);
        while (opModeIsActive() && robot.hangOne.getCurrentPosition() > -8263) {
            telemetry.addData("h1: ", "" + robot.hangOne.getCurrentPosition());
            telemetry.update();
            robot.setHangPower(-1);
        }
        robot.hangOne.setTargetPosition(robot.hangOne.getCurrentPosition());
        robot.setHangPower(0);
        resetEncoders();
        robot.hangOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                                if((recognitionNested.getLeft() + 10 > recognition.getLeft()) && (recognitionNested.getLeft() - 10 < recognition.getLeft()) && (recognitionNested.getTop() + 10 > recognition.getTop() && recognitionNested.getTop() - 10 < recognition.getTop())) {
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
                                    if(minFind.getLeft() < minRecY) {
                                        minRecY = minFind.getLeft();
                                        min1 = minFind;
                                    }
                                }
                                minRecY = Double.MAX_VALUE;
                                for(Recognition minFind : updatedRecognitions) {
                                    if(minFind.getLeft() < minRecY && minFind != min1) {
                                        minRecY = minFind.getLeft();
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
                                goldMineralX = (int) (recognition.getTop()); // Set the gold x position to its x position
                            } else if (silverMineral1X == -987654) { // If the item is silver and no silver has been assigned an x position yet....
                                silverMineral1X = (int) (recognition.getTop()); // Set the first silver x position to its x position
                            } else { // If the item is silver and another silver has been found
                                silverMineral2X = (int) (recognition.getTop()); // Set the second silver x position to its x position
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
                                goldMineralX = (int) (recognition.getTop()); // Set the gold x position to its x position (note that the phone's orientation reverses axes)
                            } else if (silverMineral1X == -987654) { // If the item is silver and no silver has been assigned an x position yet....
                                silverMineral1X = (int) (recognition.getTop()); // Set the first silver x position to its x position
                            } else {
                                silverMineral2X = (int) (recognition.getTop());
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

    /* Straighten robot to nearest 45º angle */
    private void adjustAngle() {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                robot.setDrivePower(0.1, 0.1, -0.1, -0.1);
                if (Math.abs(robotAngle) % 45 < angleOffset) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
            else {
                robot.setDrivePower(-0.1, -0.1, 0.1, 0.1);
                if (Math.abs(robotAngle) % 45 < angleOffset) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
        }
        resetEncoders();
    }

    private void adjustAngle(double time) {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Orientation g0angles = null;
        Orientation g1angles = null;
        double robotAngle;
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
                robot.setDrivePower(0.1, 0.1, -0.1, -0.1);
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
                robot.setDrivePower(-0.1, -0.1, 0.1, 0.1);
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

    private void adjustAngle(double time, double straight) {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Orientation g0angles = gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation g1angles = gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        ElapsedTime turnTime = new ElapsedTime();
        while (opModeIsActive() && turnTime.time() < time && Math.abs((g0angles.firstAngle + g1angles.firstAngle) / 2.0) % straight > angleOffset) {
            g0angles = gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            g1angles = gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2);
            if (robotAngle % straight > straight / 2) {
                robot.setDrivePower(0.1, 0.1, -0.1, -0.1);
                robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2);
                if (((Math.abs(robotAngle) % straight) < angleOffset) || turnTime.time() > time) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
            else {
                robot.setDrivePower(-0.1, -0.1, 0.1, 0.1);
                robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2);
                if (((Math.abs(robotAngle) % straight) < angleOffset) || turnTime.time() > time) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
        }
        resetEncoders();
    }

    private void adjustAngle(double time, double straight, double precision) {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Orientation g0angles = gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation g1angles = gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        ElapsedTime turnTime = new ElapsedTime();
        while (opModeIsActive() && turnTime.time() < time && Math.abs((g0angles.firstAngle + g1angles.firstAngle) / 2.0) % 45 > precision) {
            g0angles = gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            g1angles = gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2);
            if (robotAngle % straight > straight / 2) {
                robot.setDrivePower(0.1, 0.1, -0.1, -0.1);
                robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2);
                if (((Math.abs(robotAngle) % straight) < precision) || turnTime.time() > time) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
            else {
                robot.setDrivePower(-0.1, -0.1, 0.1, 0.1);
                robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2);
                if (((Math.abs(robotAngle) % straight) < precision) || turnTime.time() > time) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
        }
        resetEncoders();
    }

    /* Precise turn */
    private void turn(double angle) {
        while(angle < -180) {
            angle += 360;
        }
        while(angle > 180) {
            angle -= 360;
        }
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        double angleIntended = robotAngle + angle;
        if (angleIntended < robotAngle) { // left turn
            while(angleIntended < robotAngle) {
                robot.setDrivePower(0.05 * Math.abs(angleIntended - robotAngle), 0.05 * Math.abs(angleIntended - robotAngle), -0.05 * Math.abs(angleIntended - robotAngle), -0.05 * Math.abs(angleIntended - robotAngle));
                g0angles = null;
                g1angles = null;
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
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        else if (angleIntended > robotAngle) {
            while (angleIntended > robotAngle) {
                robot.setDrivePower(-0.05 * Math.abs(angleIntended - robotAngle), -0.05 * Math.abs(angleIntended - robotAngle), 0.05 * Math.abs(angleIntended - robotAngle), 0.05 * Math.abs(angleIntended - robotAngle));
                g0angles = null;
                g1angles = null;
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
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        resetEncoders();
    }

    private void turn(double angle, double time) {
        ElapsedTime turnTime = new ElapsedTime();
        while(angle < -180) {
            angle += 360;
        }
        while(angle > 180) {
            angle -= 360;
        }
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        double angleIntended = robotAngle + angle;
        if (angleIntended < robotAngle) { // Left turn
            while(opModeIsActive() && angleIntended < robotAngle && turnTime.time() < time) {
                robot.setDrivePower(-0.05 * Math.abs(angleIntended - robotAngle), -0.05 * Math.abs(angleIntended - robotAngle), 0.05 * Math.abs(angleIntended - robotAngle), 0.05 * Math.abs(angleIntended - robotAngle));
                g0angles = null;
                g1angles = null;
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
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        else if(opModeIsActive() && angleIntended > robotAngle && turnTime.time() < time) { // Right turn
            while(opModeIsActive() && angleIntended > robotAngle && turnTime.time() < time) {
                robot.setDrivePower(0.05 * Math.abs(angleIntended - robotAngle), 0.05 * Math.abs(angleIntended - robotAngle), -0.05 * Math.abs(angleIntended - robotAngle), -0.05 * Math.abs(angleIntended - robotAngle));
                g0angles = null;
                g1angles = null;
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
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        resetEncoders();
    }

    private void vuforiaInit() { // Initialize Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        // parameters.cameraName = hardwareMap.get(WebcamName.class, "WC1"); // Use external camera
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
        telemetry.addData("Remaining Distances", "");
        telemetry.addData("  rr: ", robot.rrWheel.getTargetPosition() - robot.rrWheel.getCurrentPosition());
        telemetry.addData("  rf: ", robot.rfWheel.getTargetPosition() - robot.rfWheel.getCurrentPosition());
        telemetry.addData("  lr: ", robot.lrWheel.getTargetPosition() - robot.lrWheel.getCurrentPosition());
        telemetry.addData("  lf: ", robot.lfWheel.getTargetPosition() - robot.lfWheel.getCurrentPosition());
        telemetry.addData("  h1: ", robot.hangOne.getTargetPosition() - robot.hangOne.getCurrentPosition());
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
        telemetry.addData("Robot angle", "" + robotAngle);
        telemetry.update();
    }

    public void run() {
        /* Reset encoders */
        if(robot.rrWheel != null && robot.rfWheel != null && robot.lfWheel != null && robot.lrWheel != null && robot.hangOne != null) {
            resetEncoders();
        }
        robot.setKickPower(kickReadyPos);

        gyro0 = robot.gyro0;
        gyro1 = robot.gyro1;

        if(robot.gyro0 != null) {
            while(!robot.gyro0.isGyroCalibrated() && opModeIsActive()) {}
        }

        if(robot.gyro1 != null) {
            while(!robot.gyro1.isGyroCalibrated() && opModeIsActive()) {}
        }

        gyro0.close();

        telemetry.addData("Message from secondary thread", "Gyros initialized");
        telemetry.update();
        Thread.currentThread().interrupt();
    }

    private void storeAngle() throws IOException {
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


    // Encoder move methods have been replaced with timed method to prevent an over-consumption of time
    // as encoders attempt to perfect some movements.  Former methods have been deprecated.

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

    // These equations used the equation x = (vix)(t) + (1/2)(ax)(t^2) for movement, assuming that initial velocity was always
    // zero.  However, due to the inaccuracy of the accelerometer, the slight delay before the motors could be fully stopped
    // after reaching the intended distance, and the difficulty of obtaining precise field distances and angular measurements,
    // encoder counts appear to be the more feasible solution.  These methods have thus been deprecated.

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
                g0accel = robot.gyro0.getAcceleration();
            }
            if (robot.gyro1 != null) {
                g1accel = robot.gyro1.getAcceleration();
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
                g0accel = robot.gyro0.getAcceleration();
            }
            if (robot.gyro1 != null) {
                g1accel = robot.gyro1.getAcceleration();
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
                g0accel = robot.gyro0.getAcceleration();
            }
            if (robot.gyro1 != null) {
                g1accel = robot.gyro1.getAcceleration();
            }
            if (g0accel != null && g1accel != null) {
                robotAccel = ((g0accel.zAccel + g1accel.zAccel) / 2);
            } else if (g0accel != null) {
                robotAccel = g0accel.zAccel;
            } else if (g1accel != null) {
                robotAccel = g1accel.zAccel;
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
                g0accel = robot.gyro0.getAcceleration();
            }
            if (robot.gyro1 != null) {
                g1accel = robot.gyro1.getAcceleration();
            }
            if (g0accel != null && g1accel != null) {
                robotAccel = ((g0accel.zAccel + g1accel.zAccel) / 2);
            } else if (g0accel != null) {
                robotAccel = g0accel.zAccel;
            } else if (g1accel != null) {
                robotAccel = g1accel.zAccel;
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
}
