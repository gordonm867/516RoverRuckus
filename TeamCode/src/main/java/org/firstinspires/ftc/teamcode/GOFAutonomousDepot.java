package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.Iterator;
import java.util.List;

// @SuppressWarnings({"WeakerAccess", "SpellCheckingInspection", "EmptyCatchBlock", "StatementWithEmptyBody", "SameParameterValue"})
@Autonomous(name="GOFAutoDepot", group="GOF")
public class GOFAutonomousDepot extends LinearOpMode {

    /* Declare OpMode members */
    private                 boolean             depot                   = false;
    private volatile        boolean             doBox                   = true;
    private volatile        boolean             doTelemetry             = true;
    private                 boolean             path                    = false;
    private                 boolean             remove;
    private volatile        boolean             threadReset             = false;
    private                 boolean             yPressed                = false;

    private                 ElapsedTime         elapsedTime             = new ElapsedTime(); // Measure timing

    private                 double              angleOffset             = 3;
    private volatile        double              boxPos                  = 111;
    private                 double              intake                  = 140;
    private                 double              neutral                 = 112;
    private volatile        double              offset                  = 2.5;
    private                 double[]            point                   = new double[2];
    private                 double              startTime               = elapsedTime.time();
    private                 double              ticksPerInch            = 560 / (4 * Math.PI);

    private volatile        GOFHardware         robot                   = GOFHardware.getInstance(); // Use the GOFHardware class

    private                 GOFVuforiaLocalizer vuforia;

    private                 int                 goldPos                 = -2;

    private volatile        OpModeManagerImpl   manager                 = (OpModeManagerImpl)this.internalOpModeServices;

    private static final    String              LABEL_GOLD_MINERAL      = "Gold Mineral";
    private static final    String              LABEL_SILVER_MINERAL    = "Silver Mineral";
    private static final    String              TFOD_MODEL_ASSET        = "RoverRuckus.tflite";
    private static final    String              VUFORIA_KEY             = "AWVhzQD/////AAABmWz790KTAURpmjOzox2azmML6FgjPO5DBf5SHQLIKvCsslmH9wp8b5zkCGfES8tt+8xslwaK7sd2h5H1jwmix26x+Eg5j60l00SlNiJMDAp5IOMWvhdJGZ8jJ8wFHCNkwERQG57JnrOXVSFDlc1sfum3oH68fEd8RrA570Y+WQda1fP8hYdZtbgG+ZDVG+9XyoDrToYU3FYl3W" + "M1iUphAbHJz1BMFFnWJdbZzOicvqah/RwXqtxRDNlem3JdT4W95kCY5bckg92oaFIBk9n01Gzg8w5mFTReYMVI3Fne72/KpPRPJwblO0W9OI3o7djg+iPjxkKOeHUWW+tmi6r3LRaKTrIUfLfazRu0QwLA8Bgw";

    private                 TFObjectDetector    detector;


    @Override
    public void runOpMode() {
        /* Initialize hardware class */
        robot.init(hardwareMap);

        /* Reset encoders */
        if (robot.rrWheel != null && robot.rfWheel != null && robot.lfWheel != null && robot.lrWheel != null) {
            robot.teamFlag.setPosition(0.05);
            robot.rrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.hangOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        point[0] = -2;
        point[1] = 2;

        vuforiaInit(); // Initialize Vuforia
        detectInit(); // Initialize TensorFlwo

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
                while(elapsedTime.time() <= 32 && doTelemetry) {
                    try {
                        String tmy = "Run Time: " + elapsedTime.toString() + "\n";
                        tmy += "Motors" + "\n";
                        tmy += "    rr: " + robot.rrWheel.getCurrentPosition() + "\n";
                        tmy += "    rf: " + robot.rfWheel.getCurrentPosition() + "\n";
                        tmy += "    lr: " + robot.lrWheel.getCurrentPosition() + "\n";
                        tmy += "    lf: " + robot.lfWheel.getCurrentPosition() + "\n";
                        tmy += "    h1: " + robot.hangOne.getCurrentPosition() + "\n";
                        tmy += "    em: " + robot.extend.getCurrentPosition() + "\n";
                        tmy += "    so: " + robot.box.getCurrentPosition() + "\n";
                        tmy += "    intake: " + (gamepad1.right_trigger) + ", " + robot.intake.getCurrentPosition() + "\n";
                        tmy += "    outtake: " + (gamepad1.left_trigger) + "\n";
                        tmy += "Servos" + "\n";
                        tmy += "    fm, actual: " + (180 * (robot.boxPotentiometer.getVoltage() / 3.3)) + "\n";
                        tmy += "    fm, intended: " + boxPos + "\n";
                        tmy += "    tm: " + robot.teamFlag.getPosition() + "\n";
                        tmy += "Gyro Data" + "\n";
                        tmy += "    Robot angle: " + getAngle() + "\n";
                        tmy += "    X acceleration: " + ((robot.gyro0.getGravity().xAccel + robot.gyro1.getGravity().xAccel) / 2) + "\n";
                        tmy += "    Y acceleration: " + ((robot.gyro0.getGravity().yAccel + robot.gyro1.getGravity().yAccel) / 2) + "\n";
                        tmy += "    Z acceleration: " + ((robot.gyro0.getGravity().zAccel + robot.gyro1.getGravity().zAccel) / 2) + "\n";
                        tmy += "Sensors" + "\n";
                        tmy += "     MR Range Sensor: " + robot.getUSDistance() + "\n";
                        tmy += "     REV 2m Distance Sensor: " + robot.getREVDistance() + "\n";
                        //tmy += "Extender limit switch voltage: " + robot.extenderSensor.getVoltage();
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
        Thread box = new Thread() {
            private ElapsedTime threadTime = new ElapsedTime();
            private double iterations = 0;
            private double integral = 0;
            private double lastError = 0;
            @Override
            public synchronized void run() {
                threadTime.reset();
                while (!doBox) {
                    try {
                        sleep(100);
                    } catch (Exception p_exception) {
                        Thread.currentThread().interrupt();
                    }
                }
                String active = null;
                try {
                    active = manager.getActiveOpModeName();
                } catch (Exception p_exception) {
                    manager = null;
                }
                while(Math.abs(robot.boxPotentiometer.getVoltage() - (3.3 * (boxPos / 180))) <= 0.0917) {
                    try {
                        sleep(50);
                    } catch (Exception p_exception) {
                        doBox = false;
                        Thread.currentThread().interrupt();
                        break;
                    }
                }
                while (elapsedTime.time() <= 32 && doBox && ((active == null || manager == null || manager.getActiveOpModeName().equalsIgnoreCase(active)))) {
                    threadTime.reset();
                    if(threadReset) {
                        threadReset = false;
                        iterations = 0;
                        integral = 0;
                    }
                    iterations++;
                    double currentAngle = 180 * (robot.boxPotentiometer.getVoltage() / 3.3);
                    double error = -(boxPos - currentAngle);
                    double derivative = 0;
                    if (iterations > 1) {
                        integral += threadTime.time() * (error - lastError);
                        derivative = threadTime.time() / (error - lastError);
                    }
                    lastError = error;
                    if (Math.abs(error) >= 5) {
                        double PIDPower;
                        if (boxPos >= 170 && currentAngle >= 170) {
                            robot.box.setPower(0);
                        } else {
                            try {
                                PIDPower = (0.03 * error) + (0.025 * integral) + (0.09 * (derivative));
                            } catch (Exception p_exception) {
                                PIDPower = (0.075 * error);
                            }
                            if (Math.abs(PIDPower) >= 0.09) {
                                robot.box.setPower(Range.clip(PIDPower, -robot.maxBoxSpeed, robot.maxBoxSpeed));
                            } else {
                                robot.box.setPower(0);
                            }
                        }
                    } else {
                        robot.box.setPower(0);
                    }
                }
            }
        };

        GOFAutoTransitioner.transitionOnStop(this, "GOFTeleOp"); // Start TeleOp after autonomous ends

        while(!gamepad1.x) {
            telemetry.addData("Double Sampling is", (path ? "ON" : "OFF") + " - Press \"Y\" to change and \"X\" to finalize (on gamepad1)");
            telemetry.update();
            if(gamepad1.y && !yPressed) {
                path = !path;
                yPressed = true;
            }
            else if(!gamepad1.y){
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
        update.start();
        box.start();

        elapsedTime.reset();
        detector.shutdown();
        //vuforia.close();

        robot.playSound(goldPos);
        if (robot.soundError) {
            telemetry.addData("Error: ", "Unable to play sound.");
        }

        /* Descend */
        flipBox(neutral);
        telemetry.addData("Thread alive?", box.isAlive());
        telemetry.update();
        descend();
        double passiveError = robot.box.getCurrentPosition();
        while(Math.abs((robot.box.getCurrentPosition() - passiveError)) <= ((1.5 * 1440) / (3 * Math.PI))) {
            robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.setDrivePower(-0.5, 0.5, 0.5, -0.5);
        }
        resetEncoders();
        robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangOne.setTargetPosition(-1560);
        robot.setHangPower(-1);
        turn(-getAngle(), 1);
        flipBox(goldPos == 0 ? 60 : neutral);
        /* Move to gold */
        if (robot.rrWheel != null && robot.rfWheel != null && robot.lfWheel != null && robot.lrWheel != null && opModeIsActive()) {
            goldPos = Range.clip(goldPos, -2, 1);
            if (goldPos == -1) {
                leftDepotAuto();
            }
            else if (goldPos == 1) {
                rightDepotAuto();
            }
            else if (goldPos == 0 || goldPos == -2) {
                centerDepotAuto();
            }
        }
        else {
            telemetry.addData("Hardware problem detected", "A wheel is null, and I'm blaming hardware. Please fix it.");
            telemetry.update();
        }

        doTelemetry = false;
        update.interrupt();
    }

    private void centerDepotAuto() {
        robot.extend.setTargetPosition(-300);
        robot.extend.setPower(1);
        flipBox(140);
        robot.setInPower(1);
        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderMovePreciseTimed(-873, -646, -202, -846, 0.3, 1);
        robot.extend.setTargetPosition(-50);
        while(robot.extend.isBusy()) {}
        resetEncoders();
        robot.setInPower(0);
        turn(-getAngle(), 1);
        flipBox(60);
        robot.setInPower(0);
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(!path) {
            while (Math.abs((robot.rrWheel.getCurrentPosition() + robot.rfWheel.getCurrentPosition() + robot.lrWheel.getCurrentPosition() + robot.lfWheel.getCurrentPosition()) / 4) >= (36 * Math.sqrt(2) * ticksPerInch)) {
                if (Math.abs(getAngle()) <= 175) {
                    double theta = Math.toRadians(-getAngle()) + Math.PI;
                    robot.setDrivePower(0.66194995074 * (Math.cos(theta) + Math.sin(theta)), 0.66194995074 * (Math.cos(theta) - Math.sin(theta)), 1 * (Math.cos(theta) - Math.sin(theta)), 1 * (Math.cos(theta) + Math.sin(theta)));
                } else {
                    robot.setDrivePower(1, 1, 1, 1);
                }
            }
            point[0] = -5;
            point[1] = -5;
            robot.teamFlag.setPosition(0.96);
            sleep(500);
            turn(-getAngle() - 45, 5);
            robot.teamFlag.setPosition(0.2);
        }
        else {
            depot = true;
            runBackToPoint(-5, -5, true);
            depot = false;
            robot.teamFlag.setPosition(0.96);
            sleep(500);
            turn(-getAngle() - 45, 5);
            robot.teamFlag.setPosition(0.2);
        }
        /*
        runToPoint(-1, -5, true);
        runToPoint(0.25,-5, true);
        encoderMovePreciseTimed(-1700, 1700, 1700, -1700, 1, 2);
        runBackToPoint(-4, -5, 0, true);
        */
        die();
        robot.extend.setTargetPosition(-3000);
        robot.extend.setPower(1);
        runToPoint(1.5, -5, 0, true);
        while(opModeIsActive() && robot.extend.isBusy()) {}
        flipBox(120);
        while(opModeIsActive()) {}
    }

    private void rightDepotAuto() {
        robot.setInPower(0);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // encoderMovePreciseTimed(-809, -143, -315, -1276, 0.3, 1);
        // resetEncoders();
        // runToPoint(-2.9, -2.9);
        point[0] = -2;
        point[1] = -2;
        telemetry.addData("Status", "Turning " + -getAngle());
        telemetry.update();
        turn(-getAngle(), 1);
        robot.setInPower(0);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // encoderMovePreciseTimed(-229, 615, 427, -201, 0.3, 1);
        // resetEncoders();
        turn(-135 - atan(-2, -0.118), 5);
        flipBox(71);
        /*
        double turns = 0;
        while(robot.centerSensor.getDistance(DistanceUnit.INCH) >= 40) {
            turn(turns < 2 ? -2 : 2, 1);
            turns++;
            if(turns > 4) {
                break;
            }
        }
        turn(-4.5, 1);
        */
        flipBox(170);
        robot.setInPower(1);
        while(opModeIsActive() && !robot.bottomSensor.isPressed() && robot.hangOne.isBusy()) {
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangOne.setPower(0);
        robot.hangOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extend.setTargetPosition(-1500);
        robot.extend.setPower(1);
        while(robot.extend.isBusy()) {}
        turn(-5, 1);
        turn(12, 1);
        turn(-7, 1);
        sample();
        robot.setInPower(1);
        resetEncoders();
        robot.extend.setTargetPosition(0);
        while(!robot.extend.isBusy()) {}
        if(path) {
            flipBox(51);
            score(point);
        }
        else {
            flipBox(71);
        }
        runToPoint(-1, -5, true);
        runToPoint(0.25,-5, true);
        encoderMovePreciseTimed(-1700, 1700, 1700, -1700, 1, 2);
        runBackToPoint(-3, -5, true);
        runToPoint(1.5, -5, true);
        robot.extend.setTargetPosition(-3000);
        robot.extend.setPower(1);
        while(opModeIsActive() && robot.extend.isBusy()) {
            robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.setDrivePower(-0.15, -0.15, -0.15, -0.15);
        }
    }

    private void leftDepotAuto() {
        robot.setInPower(0);
        // encoderMovePreciseTimed(-809, -143, -315, -1276, 0.3, 1);
        // resetEncoders();
        // runToPoint(-2.9, -2.9);
        point[0] = -2;
        point[1] = -2;
        // encoderMovePreciseTimed(-229, 615, 427, -201, 0.3, 1);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double angle;
        double newY = -3.76;
        double newX = -2.104;
        try {
            angle = atan(newY - point[1], newX - point[0]);
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = -getAngle() - 225.0 + angle;
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance, 10);
        }
        flipBox(71);
        /*
        double turns = 0;
        while(robot.centerSensor.getDistance(DistanceUnit.INCH) >= 40) {
            turn(turns < 2 ? -2 : 2, 1);
            turns++;
            if(turns > 4) {
                break;
            }
        }
        */
        flipBox(170);
        turn(-4.5, 1);
        while(opModeIsActive() && !robot.bottomSensor.isPressed() && robot.hangOne.isBusy()) {
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangOne.setPower(0);
        robot.hangOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setInPower(1);
        flipBox(170);
        robot.extend.setTargetPosition(-1000);
        robot.extend.setPower(1);
        while(robot.extend.isBusy()) {}
        turn(-5, 1);
        turn(12, 1);
        turn(-7, 1);
        sample();
        robot.setInPower(1);
        resetEncoders();
        if(path) {
            flipBox(51);
            score(point);
        }
        else {
            flipBox(71);
        }
        runToPoint(-1, -5, true);
        runToPoint(0.25,-5, true);
        encoderMovePreciseTimed(-1700, 1700, 1700, -1700, 1, 2);
        runBackToPoint(-3, -5, true);
        runToPoint(1.5, -5, true);
        robot.extend.setTargetPosition(-3000);
        robot.extend.setPower(1);
        while(opModeIsActive() && robot.extend.isBusy()) {
            robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.setDrivePower(-0.15, -0.15, -0.15, -0.15);
        }
    }

    private void die() {
        robot.box.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.box.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double roc = Double.MAX_VALUE;
        double doc = 0;
        double noc;
        while(roc > 10) {
            noc = doc;
            double first = robot.box.getCurrentPosition();
            robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.setDrivePower(0.35, -0.35, -0.35, 0.35);
            sleep(250);
            double now = robot.box.getCurrentPosition();
            roc = first - now;
            if(roc != 0) {
                doc = (first - now) / roc;
            }
            if(noc != doc && noc != 0 && doc != 0) {
                break;
            }
        }
        robot.wheelBrake();
        sleep(150);
        robot.box.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.box.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double value = 0.75 * 1440 / (3 * Math.PI);
        while(Math.abs(robot.box.getCurrentPosition()) <= value) {
            robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.setDrivePower(-0.25, 0.25, 0.25, -0.25);
            sleep(250);
        }
        resetEncoders();
    }

    private double atan(double y, double x) { // Returns atan in degrees
        return(Math.atan2(y, x) * (180 / Math.PI));
    }

    private void descend() {
        resetEncoders();
        robot.hangOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int[] ePoses = {-600, -2200, -1000};
        if(goldPos == -2) {
            goldPos = 0;
        }
        if(goldPos == 0) {
            robot.extend.setTargetPosition(ePoses[goldPos + 1]);
            robot.extend.setPower(1);
            while (goldPos == 0 && robot.extend.isBusy()) {}
            flipBox(157);
            robot.setInPower(1);
            sleep(500);
            robot.extend.setTargetPosition(-3000);
            while (robot.extend.isBusy()) {}
            sleep(1000);
            robot.setInPower(0.35);
            flipBox(neutral);
            robot.extend.setTargetPosition(0);
            while(robot.extend.isBusy()) {}
            flipBox(30);
            while(!(Math.abs(robot.box.getPower()) >= 0.08)) {}
            while(Math.abs(robot.box.getPower()) >= 0.08) {}
            flipBox(neutral);
        }
        robot.setHangPower(1);
        while(opModeIsActive() && robot.topSensor.getState()) {}
        robot.hangOne.setPower(0); // Stop sending power just in case
        robot.hangOne.setTargetPosition(robot.hangOne.getCurrentPosition()); // Set the target position to its current position to stop movement
        robot.setInPower(0);
        resetEncoders();
        robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set hang wheel back to run to position mode
        if(goldPos == 0) {
            while (robot.extend.isBusy()) {}
            robot.extend.setPower(0);
        }
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

    private void sample() {
        if(robot.intake.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            robot.intake.setPower(0);
            robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        flipBox(170);
        sleep(500);
        robot.setInPower(1);
        sleep(1000);
        if(!(goldPos == 0 && elapsedTime.time() > 15)) {
            robot.setInPower(0);
            flipBox(71);
        }
    }

    private void runToPoint(double newX, double newY, boolean reset) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]);
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = -getAngle() - 225.0 + angle;
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
        encoderMovePreciseTimed(distance, 1, Math.abs(distance / 1500.0), reset);
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runToPoint(double newX, double newY, int deviation, boolean reset) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]);
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = -getAngle() - 225.0 + angle;
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
        encoderMovePreciseTimed(distance, 1, Math.abs(distance) / 1500.0, reset);
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runToPoint(double newX, double newY, double angleError, boolean reset) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]) + angleError;
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = -getAngle() - 225.0 + angle;
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
        encoderMovePreciseTimed(distance, 1, Math.abs(distance) / 1500.0, reset);
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runToPoint(double newX, double newY, int deviation, double angleError, boolean reset) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]) + angleError;
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = -getAngle() - 225.0 + angle;
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
        encoderMovePreciseTimed(distance, 1, Math.abs(distance) / 1500, reset);
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runToPoint(double newX, double newY, float speed, boolean reset) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]);
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = -getAngle() - 225.0 + angle;
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
        encoderMovePreciseTimed(distance, speed, Math.abs(distance / 1500.0), reset);
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runBackToPoint(double newX, double newY, boolean reset) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]);
        }
        catch(Exception p_exception) {
            angle = -90;
        }
        double turnDistance = -getAngle() - 225.0 + angle + 180;
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
        encoderMovePreciseTimed(distance, 1, Math.abs(distance) / 1500.0, reset);
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runBackToPoint(double newX, double newY, int deviation, boolean reset) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]) + 180;
        }
        catch(Exception p_exception) {
            angle = -90;
        }
        double turnDistance = -getAngle() - 225.0 + angle;
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance, Math.abs(turnDistance) / 60.0);
        }
        int distance = -calculateMove(Math.abs(newX - point[0]), Math.abs(newY - point[1]), deviation);
        encoderMovePreciseTimed(distance, 1, Math.abs(distance) / 1500.0, reset);
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runBackToPoint(double newX, double newY, double angleError, boolean reset) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]) + 180 + angleError;
        }
        catch(Exception p_exception) {
            angle = -90;
        }
        double turnDistance = -getAngle() - 225.0 + angle;
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
        encoderMovePreciseTimed(distance, 1, Math.abs(distance) / 1500.0, reset);
        point[0] = newX;
        point[1] = newY;
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    private void runBackToPoint(double newX, double newY, int deviation, double angleError, boolean reset) {
        double angle;
        try {
            angle = atan(newY - point[1], newX - point[0]) + 180 + angleError;
        }
        catch(Exception p_exception) {
            angle = -90;
        }
        double turnDistance = -getAngle() - 225.0 + angle;
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance, Math.abs(turnDistance) / 60.0);
        }
        int distance = -calculateMove(Math.abs(newX - point[0]), Math.abs(newY - point[1]), deviation);
        encoderMovePreciseTimed(distance, 1, Math.abs(distance) / 1500.0, reset);
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
            while ((robot.rrWheel.isBusy() || robot.rfWheel.isBusy() || robot.lrWheel.isBusy() || robot.lfWheel.isBusy()) && opModeIsActive() && limitTest.time() < timeLimit) {}
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

    /* private void encoderMovePreciseTimed(int pos, double speed, double timeLimit) { // Move encoders towards target position until the position is reached, or the time limit expires
        double maxDrivePower = robot.maxDriveSpeed;
        robot.maxDriveSpeed = speed;
        if (opModeIsActive() && robot.rrWheel != null && robot.rfWheel != null && robot.lrWheel != null && robot.lfWheel != null) {
            robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rrWheel.setTargetPosition(pos);
            robot.rfWheel.setTargetPosition(pos);
            robot.lfWheel.setTargetPosition(pos);
            robot.lrWheel.setTargetPosition(pos);
            double startAngle = getAngle();
            ElapsedTime delta = new ElapsedTime();
            double Kp = 0.1;
            double Ki = 0.01;
            double Kd = 0.01;
            double i = 0;
            double lastError = 0;
            ElapsedTime limitTest = new ElapsedTime();
            try {
                robot.setDrivePower(-(pos / Math.abs(pos)) * speed, -(pos / Math.abs(pos)) * speed, -(pos / Math.abs(pos)) * speed, -(pos / Math.abs(pos)) * speed);
            }
            catch(Exception p_exception) {
                robot.setDrivePower(0, 0, 0, 0);
            }
            while(Math.abs(((robot.rrWheel.getCurrentPosition() + robot.rfWheel.getCurrentPosition() + robot.lrWheel.getCurrentPosition() + robot.lfWheel.getCurrentPosition()) / 4)) <= Math.abs(pos) && opModeIsActive() && limitTest.time() < timeLimit) {
                delta.reset();
                updateTelemetry();
                double angleError = getAngle() - startAngle;
                try {
                    if (Math.abs(angleError) > 180 && (Math.abs(getAngle()) / getAngle()) != (Math.abs(startAngle) / startAngle)) {
                        angleError += angleError > 0 ? -360 : 360;
                    }
                }
                catch(Exception p_exception) {} // If an error happens, that means that either our current angle or initial angle was zero, so the error calculation should be accurate anyway
                double right = getPower(robot.rrWheel);
                double left = getPower(robot.lfWheel);
                double deltaError = angleError - lastError;
                i += delta.time() * deltaError;
                double changePower = (Kp * angleError) + (Ki * i) + (Kd * deltaError / delta.time());
                right -= changePower;
                left += changePower;
                double max = Math.max(Math.abs(right), Math.max(Math.abs(left), speed));
                right /= max;
                left /= max;
                robot.setDrivePower(left, left, right, right);
                lastError = angleError;
            }
            if(limitTest.time() > timeLimit) {
                robot.rrWheel.setTargetPosition((robot.rrWheel.getCurrentPosition()));
                robot.rfWheel.setTargetPosition((robot.rfWheel.getCurrentPosition()));
                robot.lrWheel.setTargetPosition((robot.lrWheel.getCurrentPosition()));
                robot.lfWheel.setTargetPosition((robot.lfWheel.getCurrentPosition()));
            }
            robot.setDrivePower(0, 0, 0, 0);
            resetEncoders();
            robot.maxDriveSpeed = maxDrivePower;
            sleep(100);
        }
    } */

    private void encoderMovePreciseTimed(int pos, double speed, double timeLimit, boolean reset) { // Move encoders towards target position until the position is reached, or the time limit expires
        if(reset) {
            resetEncoders();
        }
        if(opModeIsActive()) {
            double maxDrivePower = robot.maxDriveSpeed;
            robot.maxDriveSpeed = speed;
            if (opModeIsActive() && robot.rrWheel != null && robot.rfWheel != null && robot.lrWheel != null && robot.lfWheel != null) {
                robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rrWheel.setTargetPosition(pos);
                robot.rfWheel.setTargetPosition(pos);
                robot.lfWheel.setTargetPosition(pos);
                robot.lrWheel.setTargetPosition(pos);
                double startAngle = getAngle();
                ElapsedTime delta = new ElapsedTime();
                double Kp = 0.043;
                double Ki = 0.01;
                double Kd = 0.02;
                double i = 0;
                double lastError = 0;
                ElapsedTime limitTest = new ElapsedTime();
                try {
                    robot.setDrivePower((pos / Math.abs(pos)) * speed, (pos / Math.abs(pos)) * speed, (pos / Math.abs(pos)) * speed, (pos / Math.abs(pos)) * speed);
                } catch (Exception p_exception) {
                    robot.setDrivePower(speed, speed, speed, speed);
                }
                while(Math.abs(((robot.rrWheel.getCurrentPosition() + robot.rfWheel.getCurrentPosition() + robot.lrWheel.getCurrentPosition() + robot.lfWheel.getCurrentPosition()) / 4)) <= Math.abs(pos) && opModeIsActive() && limitTest.time() < timeLimit && elapsedTime.time() <= 29.5) {
                    if(Math.abs(getAngle() - startAngle) >= 7) {
                        robot.rrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.rfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.lfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.lrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        double angleError = getAngle() - startAngle;
                        try {
                            if (Math.abs(angleError) > 180 && (Math.abs(getAngle()) / getAngle()) != (Math.abs(startAngle) / startAngle)) {
                                angleError += angleError > 0 ? -360 : 360;
                            }
                        }
                        catch (Exception p_exception) {} // If an error happens, that means that either our current angle or initial angle was zero, so the error calculation should be accurate anyway
                        double right = getPower(robot.rrWheel);
                        double left = getPower(robot.lfWheel);
                        double error = Math.abs(robot.rrWheel.getCurrentPosition() - robot.rrWheel.getTargetPosition()) / 500;
                        double maxSpeed = robot.maxDriveSpeed;
                        if(!(speed >= maxSpeed) && !(error <= speed)) {
                            speed += 0.05;
                        }
                        else {
                            speed = Math.min(error, Math.abs(maxSpeed));
                        }
                        try {
                            robot.setDrivePower((pos / Math.abs(pos)) * speed, (pos / Math.abs(pos)) * speed, (pos / Math.abs(pos)) * speed, (pos / Math.abs(pos)) * speed);
                        }
                        catch(Exception p_exception) {
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        double deltaError = angleError - lastError;
                        i += delta.time() * deltaError;
                        double changePower = (Kp * angleError) + (Ki * i) + (Kd * deltaError / delta.time());
                        right += changePower;
                        left -= changePower;
                        double max = Math.max(Math.abs(right), Math.max(Math.abs(left), speed));
                        right /= Math.abs(max);
                        left /= Math.abs(max);
                        robot.setDrivePower(left, left, right, right);
                        lastError = angleError;
                        delta.reset();
                        if(depot && robot.getUSDistance() <= 57 && robot.getUSDistance() >= 40) {
                            robot.setDrivePower(0, 0, 0, 0);
                            break;
                        }
                    }
                }
                if (limitTest.time() > timeLimit) {
                    robot.rrWheel.setTargetPosition((robot.rrWheel.getCurrentPosition()));
                    robot.rfWheel.setTargetPosition((robot.rfWheel.getCurrentPosition()));
                    robot.lrWheel.setTargetPosition((robot.lrWheel.getCurrentPosition()));
                    robot.lfWheel.setTargetPosition((robot.lfWheel.getCurrentPosition()));
                }
                robot.setDrivePower(0, 0, 0, 0);
                robot.maxDriveSpeed = maxDrivePower;
            }
        }
        else {
            stop();
        }
    }

    private double getPower(DcMotor motor) {
        try {
            double power = (Math.abs(motor.getPower()) / motor.getPower()) * (Math.abs(motor.getTargetPosition()) - Math.abs(motor.getCurrentPosition())) / 100;
            if(Math.abs(power) >= 0.1) {
                return(Range.clip(power, -1, 1));
            }
            else if(power != 0){
                return(0.1 * (power < 0 ? -1 : 1));
            }
            else {
                return 0;
            }
        }
        catch(Exception p_exception) {
            return 0;
        }
    }

    /*
    private double getPower(DcMotor motor) {
        try {
            double power = (Math.abs(motor.getPower()) / motor.getPower()) * (Math.abs(motor.getTargetPosition()) - Math.abs(motor.getCurrentPosition())) / 100;
            if(Math.abs(power) >= 0.1) {
                return(Range.clip(power, -1, 1));
            }
            else if(power != 0){
                return(0.1 * (power < 0 ? -1 : 1));
            }
            else {
                return 0;
            }
        }
        catch(Exception p_exception) {
            return 0;
        }
    } */

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
        robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void score(double[] actualPoint) {
        if(!(boxPos == 51)) {
            flipBox(51);
            sleep(500);
            robot.setInPower(-0.2);
        }
        else {
            robot.setInPower(0);
        }
        robot.hangOne.setTargetPosition(1400);
        robot.hangOne.setPower(1);
        robot.setInPower(0);
        runBackToPoint(-1.25, -1.25, true);
        robot.hangOne.setTargetPosition(1560);
        while(opModeIsActive() && robot.hangOne.isBusy()) {
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        robot.hangOne.setPower(0);
        sleep(1000);
        encoderMovePreciseTimed(0, 1, 1, false);
        robot.hangOne.setTargetPosition(0);
        robot.hangOne.setPower(1);
        flipBox(0.71);
        point = actualPoint;
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

    private void flipBox(double angle) {
        threadReset = true;
        boxPos = angle;
    }

    /*
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
    */


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

/*
    private void centerDepotAuto() {
        while(opModeIsActive() && !robot.bottomSensor.isPressed() && robot.hangOne.isBusy()) {
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        // SAMPLE WITH EXTENDER
        runBackToPoint(-5, -5);
        // robot.setKickPower(kickReadyPos);
        runToPoint(1.9, -5);
        // MOVE EXTENDER INTO Depot
    }

    private void rightDepotAuto() {
        turn(-getAngle() - 45 + atan(0.118, 2), 2);
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
        // SAMPLE WITH EXTENDER
        robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangOne.setTargetPosition(1560); // REPLACE WITH ACTUAL POSITION
        robot.setHangPower(1);
        runToPoint(-0.75, -0.75);
        robot.setHangPower(0);
        // robot.setKickPower(kickOutPos);
        runBackToPoint(-2, -2);
        // robot.setKickPower(kickReadyPos);
        robot.hangOne.setTargetPosition(0);
        robot.setHangPower(-1);
        runBackToPoint(-4.75, -2.162);
        runToPoint(-4.75, -5);
        robot.teamFlag.setPosition(0.96);
        while(opModeIsActive() && !robot.bottomSensor.isPressed() && robot.hangOne.isBusy()) {
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        runToPoint(1.9, -5);
        // MOVE EXTENDER INTO Depot
    }

    private void leftDepotAuto() {
        turn(-getAngle() - 45 + atan(2, 0.118), 2);
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
        // SAMPLE WITH EXTENDER
        robot.hangOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangOne.setTargetPosition(1560); // REPLACE WITH ACTUAL POSITION
        robot.setHangPower(1);
        runToPoint(-0.75, -0.75);
        robot.setHangPower(0);
        // robot.setKickPower(kickOutPos);
        runBackToPoint(-2, -2);
        // robot.setKickPower(kickReadyPos);
        robot.hangOne.setTargetPosition(0);
        robot.setHangPower(-1);
        runBackToPoint(-2.176, -5);
        turn(-getAngle() - 45 + atan(0, 2.024), 0);
        // DROP TEAM MARKER
        robot.teamFlag.setPosition(0.96);
        // RETRACT TEAM MARKER
        while(opModeIsActive() && !robot.bottomSensor.isPressed() && robot.hangOne.isBusy()) {
            double oldPos = robot.hangOne.getCurrentPosition();
            sleep(100);
            double newPos = robot.hangOne.getCurrentPosition();
            if(oldPos == newPos) {
                break;
            }
        }
        runToPoint(1.9, -5);
        // MOVE EXTENDER INTO Depot
    }
    */