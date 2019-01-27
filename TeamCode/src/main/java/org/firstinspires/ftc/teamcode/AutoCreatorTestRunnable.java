package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import android.util.Base64;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
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

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.Iterator;
import java.util.List;

@Autonomous(name="AutoCreatorRunner",group="GOFTests")
@Disabled

public class AutoCreatorTestRunnable extends LinearOpMode {
    private                 boolean                 aPressed                = false;
    private volatile        boolean                 doTelemetry             = true;
    private                 boolean                 doubleSample            = true;
    private                 boolean                 crater                  = false;
    private                 boolean                 ypressed                = false;
    private                 boolean                 yPressedInit            = false;
    private                 boolean                 bumperPressed           = false;
    private                 boolean                 servoMove               = false;
    private                 boolean                 remove;


    private                 double                  angle;
    private                 double                  drive;
    private                 double                  firstAngleOffset;
    private                 double                  lastIntake              = 0;
    private                 double                  maxDriveSpeed;
    private                 double                  turn;

    private                 ElapsedTime             elapsedTime             = new ElapsedTime();

    public                  GOFHardware             robot                   = GOFHardware.getInstance(); // Use the GOFHardware class

    private                 int                     driverMode              = 1;
    private                 int                     goldPos                 = -2;

    private static final    String              TFOD_MODEL_ASSET            = "RoverRuckus.tflite";
    private static final    String              LABEL_GOLD_MINERAL          = "Gold Mineral";
    private static final    String              LABEL_SILVER_MINERAL        = "Silver Mineral";
    private static final    String              VUFORIA_KEY                 = "AWVhzQD/////AAABmWz790KTAURpmjOzox2azmML6FgjPO5DBf5SHQLIKvCsslmH9wp8b5zkCGfES8tt+8xslwaK7sd2h5H1jwmix26x+Eg5j60l00SlNiJMDAp5IOMWvhdJGZ8jJ8wFHCNkwERQG57JnrOXVSFDlc1sfum3oH68fEd8RrA570Y+WQda1fP8hYdZtbgG+ZDVG+9XyoDrToYU3FYl3WM1iUphAbHJz1BMFFnWJdbZzOicvqah/RwXqtxRDNlem3JdT4W95kCY5bckg92oaFIBk9n01Gzg8w5mFTReYMVI3Fne72/KpPRPJwblO0W9OI3o7djg+iPjxkKOeHUWW+tmi6r3LRaKTrIUfLfazRu0QwLA8Bgw";

    private                 GOFVuforiaLocalizer vuforia;
    private                 TFObjectDetector    detector;

    private                 double              startTime                   = elapsedTime.time();

    public void runOpMode() {
        /* Initialize hardware class */
        msStuckDetectInit = 10000; // Allow gyros to calibrate
        robot.init(hardwareMap);
        // robot.setKickPower(kickReadyPos);
        robot.teamFlag.setPosition(0.026);
        maxDriveSpeed = robot.maxDriveSpeed;
        telemetry.addData("Status", "Initialized"); // Update phone
        /* Reset encoders */
        if (robot.rrWheel != null && robot.rfWheel != null && robot.lfWheel != null && robot.lrWheel != null) {
            robot.teamFlag.setPosition(0.420);
            robot.rrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.hangOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        while(!gamepad1.x) {
            telemetry.addData("Double Sampling is", (doubleSample ? "ON" : "OFF") + " - Press \"Y\" to change and \"X\" to finalize (on gamepad1)");
            telemetry.update();
            if(gamepad1.y && !yPressedInit) {
                doubleSample = !doubleSample;
                yPressedInit = true;
            }
            else {
                yPressedInit = false;
            }
        }
        while(gamepad1.x) {}
        while(!gamepad1.x) {
            telemetry.addData("This autonomous path begins in front of the ", (crater ? "CRATER" : "DEPOT") + " - Press \"Y\" to change and \"X\" to finalize (on gamepad1)");
            telemetry.update();
            if(gamepad1.y && !yPressedInit) {
                crater = !crater;
                yPressedInit = true;
            }
            else {
                yPressedInit = false;
            }
        }
        int goldPos = detectGold();
        BufferedReader reader;
        try {
            reader = new BufferedReader(new FileReader(Environment.getExternalStorageDirectory().getPath() + File.separator + (crater ? (doubleSample ? "GOFCratAutoDS" : "GOFCratAutoSS") : "GOFDepAuto") + ((goldPos == 0) ? "Center" : (goldPos == 1) ? "Right" : "Left") + ".txt"));
        }
        catch(Exception all) {
            throw new RuntimeException("This doesn't work.");
        }
        waitForStart();
        Thread update = new Thread() {
            @Override
            public synchronized void run() {
                while(!doTelemetry) {
                    try {
                        sleep(100);
                    }
                    catch(Exception p_exception) {
                        Thread.currentThread().interrupt();
                    }
                }
                while(doTelemetry) {
                    try {
                        String tmy = "Run Time: " + elapsedTime.toString() + "\n";
                        tmy += "Motors" + "\n";
                        tmy += "    rr: " + robot.rrWheel.getCurrentPosition() + "\n";
                        tmy += "    rf: " + robot.rfWheel.getCurrentPosition() + "\n";
                        tmy += "    lr: " + robot.lrWheel.getCurrentPosition() + "\n";
                        tmy += "    lf: " + robot.lfWheel.getCurrentPosition() + "\n";
                        tmy += "    h1: " + robot.hangOne.getCurrentPosition() + "\n";
                        tmy += "    em: " + robot.extend.getCurrentPosition() + "\n";
                        tmy += "    intake: " + (gamepad1.right_trigger) + ", " + robot.intake.getCurrentPosition() + "\n";
                        tmy += "    outtake: " + (gamepad1.left_trigger) + "\n";
                        tmy += "Servos" + "\n";
                        tmy += "    fm: " + robot.box.getPosition() + "\n";
                        tmy += "    tm: " + robot.teamFlag.getPosition() + "\n";
                        tmy += (driverMode == 1 ? "Drive Mode: Normal" : driverMode == -1 ? "Drive Mode: Field-Oriented" : "Drive Mode: Null") + "\n";
                        tmy += "Gyro Data" + "\n";
                        tmy += "    Robot angle: " + getAngle() + "\n";
                        tmy += "    X acceleration" + ((robot.gyro0.getGravity().xAccel + robot.gyro1.getGravity().xAccel) / 2) + "\n";
                        tmy += "    Y acceleration" + ((robot.gyro0.getGravity().yAccel + robot.gyro1.getGravity().yAccel) / 2) + "\n";
                        tmy += "    Z acceleration" + ((robot.gyro0.getGravity().zAccel + robot.gyro1.getGravity().zAccel) / 2) + "\n";
                        tmy += "Variables" + "\n";
                        tmy += "    Drive: " + drive + "\n";
                        tmy += "    Turn: " + turn + "\n";
                        tmy += "    Angle: " + angle + "\n";
                        telemetry.addData("", tmy);
                    } catch (Exception p_exception) {
                        telemetry.addData("Uh oh", "The driver controller was unable to communicate via telemetry.  For help, please seek a better programmer.");
                    }
                    telemetry.update();
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
        };
        update.start();
        while(opModeIsActive()) {
            Gamepad gamepadV1 = new Gamepad();
            try {
                String getPad1 = reader.readLine();
                if(getPad1.equalsIgnoreCase("\n") || getPad1.equalsIgnoreCase(" ") || getPad1.isEmpty()) {
                    getPad1 = reader.readLine();
                }
                gamepadV1.fromByteArray(Base64.decode(getPad1, Base64.NO_WRAP));
            }
            catch(Exception noWorkException) {
                giveUp();
            }
            Gamepad gamepadV2 = new Gamepad();
            try {
                String getPad2 = reader.readLine();
                if(getPad2.equalsIgnoreCase("\n") || getPad2.equalsIgnoreCase(" ") || getPad2.isEmpty()) {
                    getPad2 = reader.readLine();
                }
                gamepadV2.fromByteArray(Base64.decode(getPad2, Base64.NO_WRAP));
            }
            catch(Exception noWorkException) {
                giveUp();
            }
            drive(gamepadV1, gamepadV2);
        }
        robot.wheelBrake();
        robot.hangBrake();
        robot.extend.setPower(0);
        try {
            reader.close();
        }
        catch(Exception notCloseableException) {
            telemetry.addData("Note", "The file could not be closed; this may or may not affect things");
            telemetry.update();
        }
    }

    private void giveUp() {
        throw new RuntimeException("I give up");
    }

    private void drive(Gamepad gamepadV1, Gamepad gamepadV2) {
        drive = gamepadV1.left_stick_y;
        double hangDrive = -gamepadV2.left_stick_y;
        turn = -gamepadV1.right_stick_x;
        angle = -gamepadV1.left_stick_x;

        /* Precision vertical drive */
        if (gamepadV1.dpad_down || gamepadV1.dpad_up) {
            if (gamepadV1.left_stick_y != 0) {
                drive = drive * 0.2; // Slow down joystick driving
            } else {
                if (gamepadV1.dpad_down) {
                    drive = 0.2; // Slow drive backwards
                } else {
                    drive = -0.2; // Slow drive forwards
                }
            }
        }

        /* Precision sideways drive */
        if (gamepadV1.dpad_right || gamepadV1.dpad_left) {
            if (gamepadV1.right_stick_x != 0) {
                angle = angle * 0.3; // Slow down joystick side movement
            } else {
                if (gamepadV1.dpad_left) {
                    angle = 0.3; // Slow leftwards
                } else {
                    angle = -0.3; // Slow rightwards
                }
            }
        }

        /* Precision turn */
        if (gamepadV1.left_bumper) {
            turn = 0.2; // Slow left turn
        }
        if (gamepadV1.right_bumper) {
            turn = -0.2; // Slow right turn
        }

        if(driverMode == 1) {
            drive = adjust(drive);
            turn = adjust(turn);
            angle = adjust(angle);
            double scaleFactor;
            if (Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))) > 1) {
                scaleFactor = maxDriveSpeed / (Math.max(Math.abs(drive + turn + angle), Math.abs(drive - turn - angle)));
            } else {
                scaleFactor = 1;
            }
            robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)); // Set motors to values based on gamepad
        }
        else if(driverMode == -1) {
            driveByField(drive, turn, angle);
        }
        else {
            driverMode = 1;
        }

        if((gamepadV1.right_trigger - gamepadV1.left_trigger) != 0 || gamepadV2.dpad_up || gamepadV2.dpad_down) {
            robot.setInPower(gamepadV1.right_trigger + (gamepadV2.dpad_up ? 1 : 0) - gamepadV1.left_trigger - (gamepadV2.dpad_down ? 1 : 0)); // Set intake power based on the gamepad trigger values
            lastIntake = (robot.intake.getCurrentPosition() - lastIntake);
            lastIntake /= (lastIntake == 0 ? 1 : Math.abs(lastIntake));
        }
        else {
            robot.setInPower(0);
            /*
            try {
                if(lastIntake != 0 && robot.intake.getCurrentPosition() % 288 != 0 && !robot.intake.isBusy()) {
                    try {
                        robot.setInPos((int)((288 * (robot.intake.getCurrentPosition() / 288)) + (288 * lastIntake)), 1);
                    }
                    catch(Exception p_exception) {
                        robot.setInPower(0);
                    }
                }
                else {
                    robot.setInPower(0);
                }
            }
            catch (Exception p_exception) {
                telemetry.addData("Note: ", "I hope you weren't planning on using intake, since it's not really working.");
            } */
        }
        /*
        if (gamepadV2.b) {
            bpressedtwo = true;
        }

        if (bpressedtwo && !gamepadV2.b) { // Toggle sorting system
            inReady = Math.abs(inReady - 1); // If inReady = 0, set it to 1; otherwise, set it to 0
            forcedOn = false;
            bpressedtwo = false;
        }

        if (gamepadV2.a) {
            apressedtwo = true;
        }

        if (apressedtwo && !gamepadV2.a) { // Force sorting system on until turned off manually
            forcedOn = true;
            bpressedtwo = false;
        }

        if (inReady == 1 && robot.frontDistanceSensor != null && robot.backDistanceSensor != null) {
            // robot.setKickPower(kickReadyPos); // Move kicker out of the way
            // robot.setDoorPower(doorOpenPos); // Open intake

            if (robot.frontDistanceSensor != null && robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 1) {
                frontCube = 0;
            } else {
                try { // Possible division by zero error
                    inRatioOne = robot.frontColorSensor.red() / robot.frontColorSensor.blue(); // GOLD: RED TO BLUE = 2:1, SILVER: RED TO BLUE = 1:1, PURPLE: RED TO BLUE 1:2
                } catch (Exception p_exception) {
                    inRatioOne = 0;
                }
                if (Math.abs((2 - inRatioOne)) > Math.abs((1 - inRatioOne))) { // Silver
                    frontCube = 1;
                } else { // Gold
                    frontCube = 2;
                }
            }
            if (robot.backDistanceSensor != null && robot.backDistanceSensor.getDistance(DistanceUnit.INCH) > 1) {
                backCube = 0;
            } else {
                try {
                    inRatioTwo = robot.backColorSensor.red() / robot.backColorSensor.blue();
                } catch (Exception p_exception) {
                    inRatioTwo = 0;
                }
                if (Math.abs((2 - inRatioTwo)) > Math.abs((1 - inRatioTwo))) { // Silver
                    backCube = 1;
                } else { // Gold
                    backCube = 2;
                }
            }

            if ((backCube * frontCube != 0) && (backCube == frontCube) && !forcedOn) {
                inReady = 0;
            } else if (backCube * frontCube != 0) {
                // robot.setKickPower(kickOutPos); // Kick mineral out of container
                frontCube = 0; // the front cube should be not there
            }
        }
        */

        if (gamepadV1.y) {
            ypressed = true;
        }

        if (ypressed && !gamepadV1.y) {
            ypressed = false;
            driverMode *= -1;
        }

        /* Reset encoders */
        if (gamepadV1.a && !gamepadV1.start) {
            robot.rrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.hangOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hangOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            firstAngleOffset = 0;
            robot.gyroInit();
        }

        if (gamepadV2.dpad_left || gamepadV2.dpad_right) {
            if (gamepadV2.left_stick_y != 0) {
                hangDrive = hangDrive * 0.25; // Slow down joystick hanging
            } else {
                if (gamepadV2.dpad_right) {
                    hangDrive = 0.25; // Slow drive hanging
                } else {
                    hangDrive = -0.25; // Slow drive hanging
                }
            }
        }
        robot.setHangPower(hangDrive); // Move container based on gamepad positions
        if(!servoMove) {
            robot.setExtendPower((Math.abs(gamepadV2.right_stick_x) < 0.05 ? gamepadV2.dpad_right ? 0.25 : gamepadV2.dpad_left ? -0.25 : gamepadV1.x ? -1 : gamepadV1.b ? 1 : 0 : gamepadV2.right_stick_x));
        }

        if(((Math.abs(gamepadV2.right_stick_x) < 0.05 ? gamepadV2.dpad_right ? 0.25 : gamepadV2.dpad_left ? -0.25 : gamepadV1.x ? -1 : gamepadV1.b ? 1 : 0 : gamepadV2.right_stick_x)) != 0) {
            servoMove = false;
        }

        if(gamepadV2.left_trigger != 0) {
            robot.flipBox(0.535);
        }
        if(gamepadV2.right_trigger != 0) {
            robot.flipBox(0.59);
        }
        if(gamepadV2.right_bumper && !bumperPressed) {
            robot.flipBox(0.414);
        }
        if(bumperPressed && !(gamepadV2.right_bumper || gamepadV2.left_bumper)) {
            bumperPressed = false;
        }
        if(gamepadV2.a && !aPressed) {
            robot.flipBox(0.535);
            aPressed = true;
            servoMove = true;
        }
        if(aPressed && !gamepadV1.a) {
            aPressed = false;
        }
        if(servoMove && !(robot.extenderSensor.getVoltage() > 2)) {
            robot.setExtendPower(1);
        }
        if(robot.extenderSensor.getVoltage() > 2 && servoMove) {
            robot.setExtendPower(0);
            servoMove = false;
            robot.flipBox(0.414);
        }
    }

    private void driveByField(double drive, double turn, double angle) { // Experimental field-oriented drive
        try {
            if(Math.round(10 * drive) == 0 && Math.round(10 * angle) == 0) {
                robot.setDrivePower(turn, turn, -turn, -turn);
            }
            else {
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
                    robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2) + firstAngleOffset; // Average angle measures to determine actual robot angle
                }
                else if (g0angles != null) {
                    robotAngle = g0angles.firstAngle + firstAngleOffset;
                }
                else if (g1angles != null) {
                    robotAngle = g1angles.firstAngle + firstAngleOffset;
                }
                else {
                    telemetry.addData("Note", "As the gyros are not working, field-centric driving has been disabled, and direct drive will be used regardless of driver-controlled settings");
                    telemetry.update();
                    robot.setDrivePower(drive + turn - angle, drive + turn + angle, drive - turn + angle, drive - turn - angle);
                    return;
                }
                double x = Math.sqrt(Math.pow(drive, 2) + Math.pow(angle, 2)); // Hypotenuse for right triangle representing robot movement
                double theta = -robotAngle + Math.atan2(drive, angle);
                drive = x * Math.sin(theta); // Set forward speed dependent on the intended angle of movement, adjusting for the angle of the robot
                angle = x * Math.cos(theta); // Set sideways speed dependent on the intended angle of movement, adjusting for the angle of the robot
                double scaleFactor = maxDriveSpeed / Math.max(maxDriveSpeed, Math.max(Math.max(Math.abs(drive + turn + angle), Math.abs(drive + turn - angle)), Math.max(Math.abs(drive - turn - angle), Math.abs(drive - turn + angle))));
                robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)); // Send adjusted values to GOFHardware() class
            }
        }
        catch(Exception p_exception) {
            robot.setDrivePower(turn, turn, -turn, -turn);
        }
    }

    private double adjust(double varToAdjust) { // Square-root driving
        if(varToAdjust < 0) {
            varToAdjust = -Math.sqrt(-varToAdjust);
        }
        else {
            varToAdjust = Math.sqrt(varToAdjust);
        }
        return varToAdjust;
    }

    private int detectGold() {
        vuforiaInit();
        detectInit();
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
                                telemetry.addData("Status", "Filtering out Depot.... (this is really bad since you're not actually running Depot auto, by the way)");
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
                            telemetry.addData("Error", "The Depot is in the frame and could not be filtered.  Please adjust the camera accordingly");
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
}
