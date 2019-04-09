package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// @SuppressWarnings({"WeakerAccess", "SpellCheckingInspection", "EmptyCatchBlock", "StatementWithEmptyBody", "SameParameterValue"})
@TeleOp(name="GOFTeleOp", group="GOF")
public class GOFBoxPotentiometerPoses extends OpMode {

    private             boolean             aPressed            = false;
    private             boolean             ypressed            = false;
    private             boolean             bumperPressed       = false;
    private             boolean             gotThere            = false;
    private             boolean             servoMove           = false;
    private             boolean             hanging             = true;
    private             boolean             hangDown            = false;

    private volatile    double              boxPos              = 120;
    private             double              firstAngleOffset;
    private             double              triggerPressed      = 0;
    private             double              lastIntake          = 0;
    private             double              integral            = 0;
    private             double              lastError           = 0;
    private             double              maxDriveSpeed;
    private             double              dump                = 30;
    private             double              intake              = 128.76;
    private             double              neutral             = 65;
    private             double              offset              = 2.5;
    private             double              Kp                  = 0.03;
    private             double              Ki                  = 0.0075;
    private             double              Kd                  = 0.015;

    private             ElapsedTime         hangTime            = new ElapsedTime();
    private volatile    ElapsedTime         threadTime          = new ElapsedTime();
    private volatile    ElapsedTime         elapsedTime         = new ElapsedTime();
    private             ElapsedTime         lightsTime          = new ElapsedTime();

    public              GOFHardware         robot               = GOFHardware.getInstance(); // Use the GOFHardware class

    private static      GOFTeleOp           teleOp              = GOFTeleOp.getInstance();

    private             int                 driverMode          = 1;
    private             int                 iterations          = 0;

    @Override
    public void init() {
        msStuckDetectInit = 10000; // Allow gyros to calibrate
        robot.init(hardwareMap);
        robot.extend.setDirection(DcMotorSimple.Direction.REVERSE);
        // robot.setKickPower(kickReadyPos);
        if(robot.teamFlag != null) {
            robot.teamFlag.setPosition(0.05);
        }
        maxDriveSpeed = robot.maxDriveSpeed;
        if(robot.boxPotentiometer != null) {
            boxPos = 180 * (robot.boxPotentiometer.getVoltage() / 3.3);
        }
        telemetry.addData("Status", "Initialized"); // Update phone
    }

    @Override
    public void init_loop() {
        checkBox();
        iterations++;
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        elapsedTime.reset();
        double drive = -gamepad1.left_stick_y;
        double hangDrive = -gamepad2.left_stick_y;
        double turn = gamepad1.right_stick_x * 0.6;
        double angle = gamepad1.left_stick_x;

        /* Precision vertical drive */
        if (gamepad1.dpad_down || gamepad1.dpad_up) {
            if (gamepad1.left_stick_y != 0) {
                drive = drive * 0.2; // Slow down joystick driving
            } else {
                if (gamepad1.dpad_down) {
                    drive = -0.2; // Slow drive backwards
                } else {
                    drive = 0.2; // Slow drive forwards
                }
            }
        }

        /* Precision sideways drive */
        if (gamepad1.dpad_right || gamepad1.dpad_left) {
            if (gamepad1.right_stick_x != 0) {
                angle = angle * 0.3; // Slow down joystick side movement
            } else {
                if (gamepad1.dpad_left) {
                    angle = -0.3; // Slow leftwards
                } else {
                    angle = 0.3; // Slow rightwards
                }
            }
        }

        /* Precision turn */
        if (gamepad1.left_bumper) {
            turn = -0.2; // Slow left turn
        }
        if (gamepad1.right_bumper) {
            turn = 0.2; // Slow right turn
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
            scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), 0.2);
            robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)); // Set motors to values based on gamepad
        }
        else if(driverMode == -1) {
            driveByField(drive, turn, angle);
        }
        else {
            driverMode = 1;
        }

        if(gamepad1.left_trigger != 0) {
            triggerPressed = gamepad1.left_trigger;
        }

        if(gamepad1.left_trigger == 0 && triggerPressed != 0) {
            triggerPressed = 0;
        }

        if((triggerPressed != 0 || gamepad2.dpad_up || gamepad2.dpad_down || servoMove) || gamepad1.a) {
            robot.setInPower((gamepad2.dpad_up ? 1 : 0) + triggerPressed - (gamepad2.dpad_down ? 1 : 0) + (servoMove ? 0.4 : 0) - (gamepad1.a ? 1 : 0)); // Set intake power based on the gamepad trigger values
            lastIntake = (robot.intake.getCurrentPosition() - lastIntake);
            lastIntake /= (lastIntake == 0 ? 1 : Math.abs(lastIntake));
        }
        else {
            robot.setInPower(0);
        }

        if (gamepad1.y && !gamepad1.start) {
            ypressed = true;
        }

        if (ypressed && !gamepad1.y) {
            ypressed = false;
            driverMode *= -1;
        }

        /* Reset encoders */
        if (gamepad1.y && gamepad1.start) {
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

        if (gamepad2.dpad_left || gamepad2.dpad_right) {
            if (gamepad2.left_stick_y != 0) {
                hangDrive = hangDrive * 0.25; // Slow down joystick hanging
            } else {
                if (gamepad2.dpad_right) {
                    hangDrive = 0.25; // Slow drive hanging
                } else {
                    hangDrive = -0.25; // Slow drive hanging
                }
            }
        }

        if(gamepad2.x && robot.bottomSensor != null && !robot.bottomSensor.isPressed()) {
            hangDown = true;
        }

        if(hangDown) {
            robot.setHangPower(-1);
        }

        if(hangDrive >= 0.25) {
            hangDown = false;
        }

        if(robot.bottomSensor != null && robot.bottomSensor.isPressed() && hangDown) {
            robot.setHangPower(0);
            hangDown = false;
        }

        if(robot.hangOne != null && robot.hangOne.getPower() > 0.05 && boxPos == dump) {
            flipBox(100);
        }

        if(!servoMove && !hangDown) {
            robot.setHangPower(hanging ? hangDrive : 0); // Move container based on gamepad positions
        }

        if(!servoMove) {
            double extendDrive = (Math.abs(gamepad2.right_stick_x) < 0.05 ? gamepad2.dpad_right ? 0.25 : gamepad2.dpad_left ? -0.25 : gamepad1.x ? -1 : gamepad1.b ? 1 : 0 : gamepad2.right_stick_x);
            robot.setExtendPower(extendDrive);
        }

        if(((Math.abs(gamepad2.right_stick_x) < 0.05 ? gamepad2.dpad_right ? 0.25 : gamepad2.dpad_left ? -0.25 : gamepad1.x ? -1 : gamepad1.b ? 1 : 0 : gamepad2.right_stick_x)) != 0) {
            servoMove = false;
        }

        if(gamepad2.left_trigger != 0) {
            flipBox(neutral); // Neutral
        }

        if(gamepad2.right_trigger != 0) {
            flipBox(intake); // Intake
        }

        if(gamepad2.left_bumper) {
            flipBox(100);
        }

        if(gamepad2.right_bumper && !bumperPressed) {
            flipBox(dump); // Dump
            hangTime.reset();
            hanging = false;
        }

        iterations++;
        checkBox();

        if(gamepad2.y) {
            Ki = 0.005;
            Kd = 0.025;
        }

        if(gamepad2.b) {
            Ki = 0;
            Kd = 0;
        }

        if(bumperPressed && !(gamepad2.right_bumper || gamepad2.left_bumper)) {
            bumperPressed = false;
        }

        if(gamepad2.a && !gamepad2.start && !aPressed) {
            flipBox(65);
            aPressed = true;
            servoMove = true;
            robot.setInPower(0.25);
        }

        if(aPressed && !gamepad1.a) {
            aPressed = false;
        }

        if(servoMove && robot.extend != null && !(robot.extenderSensor.getVoltage() > 2)) {
            robot.setExtendPower(1);
        }

        if(servoMove && robot.bottomSensor != null && !(robot.bottomSensor.isPressed())) {
            robot.setHangPower(-1);
        }

        if(robot.extenderSensor != null && robot.extenderSensor.getVoltage() > 2 && servoMove) {
            robot.setExtendPower(0);
            if(robot.bottomSensor != null && robot.bottomSensor.isPressed()) {
                robot.setHangPower(0);
                servoMove = false;
                flipBox(dump);
            }
        }

        if(!servoMove && robot.extend.getPower() == 0) {
            robot.extend.setPower(-0.1);
        }

        if(robot.bottomSensor != null && robot.bottomSensor.isPressed() && servoMove) {
            robot.setHangPower(0);
            if(robot.extenderSensor.getVoltage() > 2) {
                servoMove = false;
                robot.extend.setPower(0);
                flipBox(dump);
            }
        }

        if(servoMove && robot.bottomSensor != null && robot.bottomSensor.isPressed() && robot.extenderSensor != null && robot.extenderSensor.getVoltage() > 2) {
            robot.setHangPower(0);
            robot.setExtendPower(0);
            servoMove = false;
            flipBox(dump);
        }

        if(hangTime.time() >= 0.75) {
            hanging = true;
        }

        if(robot.lights != null) {
            if (lightsTime.time() >= 100 && lightsTime.time() <= 110) {
                if (robot.lights != null && (iterations / 90) % 2 == 0) {
                    robot.lights.pixels[0].setRGB(60, 0, 60);
                    robot.lights.pixels[2].setRGB(60, 0, 60);
                } else if (robot.lights != null) {
                    robot.lights.pixels[0].setRGB(0, 0, 0);
                    robot.lights.pixels[2].setRGB(0, 0, 0);
                }
            } else if (lightsTime.time() >= 110 && lightsTime.time() <= 115) {
                if (robot.lights != null && (iterations / 90) % 2 == 0) {
                    robot.lights.pixels[0].setRGB(60, 0, 0);
                    robot.lights.pixels[2].setRGB(60, 0, 0);
                } else if (robot.lights != null) {
                    robot.lights.pixels[0].setRGB(0, 0, 0);
                    robot.lights.pixels[2].setRGB(0, 0, 0);
                }
            } else if (lightsTime.time() >= 115) {
                if ((iterations / 45) % 2 == 0) {
                    robot.lights.pixels[0].setRGB(255, 0, 0);
                    robot.lights.pixels[2].setRGB(255, 0, 0);
                } else if (robot.lights != null) {
                    robot.lights.pixels[0].setRGB(0, 0, 0);
                    robot.lights.pixels[2].setRGB(0, 0, 0);
                }
            }
            robot.lights.update();
        }
        iterations++;
        checkBox();
    }

    @Override
    public void stop() { // Run when "STOP" pressed
        robot.enabled = false;
        robot.wheelBrake();
        robot.hangBrake();
        // robot.setKickPower(kickReadyPos); // Move kick servo to "intake ready" position
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

    private void flipBox(final double angle) {
        iterations = 0;
        integral = 0;
        boxPos = angle;
        gotThere = false;
    }

    private void checkBox() {
        if(robot.box != null) {
            if(boxPos >= 115) {
                robot.box.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                robot.box.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            robot.box.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double currentAngle = 180 * (robot.boxPotentiometer.getVoltage() / 3.3);
            double error = -(boxPos - currentAngle);
            double derivative = 0;
            if (Math.abs(error) >= offset && !gotThere) {
                if (iterations > 1) {
                    integral += threadTime.time() * error;
                    derivative = (error - lastError) / threadTime.time();
                    if (Math.abs(integral) >= 50) {
                        integral = 50 * Math.signum(integral);
                    }
                }
                lastError = error;
                double PIDPower;
                try {
                    PIDPower = (Kp * error) - (Ki * integral) + (Kd * (derivative));
                } catch (Exception p_exception) {
                    PIDPower = (Kp * error);
                }
                if (Math.abs(PIDPower) >= 0.15) {
                    if ((currentAngle <= 175 || PIDPower <= 0) && (currentAngle >= 20 || PIDPower >= 0)) {
                        robot.box.setPower(Range.clip(PIDPower, -robot.maxBoxSpeed * (7.0 / 12.0), robot.maxBoxSpeed));
                        threadTime.reset();
                    } else {
                        robot.box.setPower(Range.clip(PIDPower, -robot.maxBoxSpeed * (7.0 / 12.0), robot.maxBoxSpeed));
                    }
                } else {
                    robot.box.setPower(0);
                }
            } else {
                robot.box.setPower(0);
                gotThere = true;
            }
        }
    }

    public static GOFTeleOp getInstance() {
        if(teleOp == null) {
            teleOp = new GOFTeleOp();
        }
        return teleOp;
    }

    /*
    private void doTelemetry() {
        try {
            String tmy = "Run Time: " + elapsedTime.toString() + "\n";
            tmy += "Motors" + "\n";
            tmy += "    rr: " + robot.rrWheel.getCurrentPosition() + "\n";
            tmy += "    rf: " + robot.rfWheel.getCurrentPosition() + "\n";
            tmy += "    lr: " + robot.lrWheel.getCurrentPosition() + "\n";
            tmy += "    lf: " + robot.lfWheel.getCurrentPosition() + "\n";
            tmy += "    h1: " + robot.hangOne.getCurrentPosition() + "\n";
            tmy += driverMode == 1 ? "Drive Mode: Normal" : driverMode == -1 ? "Drive Mode: Field-Oriented" : "Drive Mode: Null";
            tmy += "Robot angle: " + getAngle() + "\n";
            tmy += "Drive: " + drive + "\n";
            tmy += "Turn: " + turn + "\n";
            tmy += "Angle: " + angle + "\n";
            tmy += "Intake: " + (gamepad1.right_trigger) + "\n";
            tmy += "Outtake: " + (gamepad1.left_trigger) + "\n";
            tmy += "X acceleration" + ((robot.gyro0.getGravity().xAccel + robot.gyro1.getGravity().xAccel) / 2) + "\n";
            tmy += "Y acceleration" + ((robot.gyro0.getGravity().yAccel + robot.gyro1.getGravity().yAccel) / 2) + "\n";
            tmy += "Z acceleration" + ((robot.gyro0.getGravity().zAccel + robot.gyro1.getGravity().zAccel) / 2) + "\n";
            tmy += "Cycle Time: " + timeDifference;
            telemetry.addData("", tmy);
        } catch (Exception p_exception) {
            telemetry.addData("Uh oh", "The driver controller was unable to communicate via telemetry.  For help, please seek a better programmer.");
        }
        telemetry.update();
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
    */

}