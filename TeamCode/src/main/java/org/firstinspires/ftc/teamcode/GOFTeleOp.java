package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="GOFTeleOp", group="GOF")

public class GOFTeleOp extends OpMode {

    /* Declare OpMode members */
    private     boolean             autoIntake          = true;
    private     boolean             bpressed            = false;
    private     boolean             doTelemetry         = true;
    private     boolean             ypressed            = false;

    private     double              angle;
    private     double              drive;
    private     double              endTime;
    private     double              firstAngleOffset;
    private     double              kickOutPos          = 0.35;
    private     double              kickReadyPos        = 0.175;
    private     double              maxDriveSpeed;
    private     double              startingTime;
    private     double              timeDifference;
    private     double              turn;

    private     ElapsedTime         elapsedTime         = new ElapsedTime();

    public      GOFHardware         robot               = GOFHardware.getInstance(); // Use the GOFHardware class

    private     int                 driverMode          = 1;

    @Override
    public void init() {
        if(!robot.inited) {
            msStuckDetectInit = 10000; // Allow gyros to calibrate
            robot.init(hardwareMap);
        }
        robot.setKickPower(kickReadyPos);
        robot.teamFlag.setPosition(0.02);
        maxDriveSpeed = robot.maxDriveSpeed;
        telemetry.addData("Status", "Initialized"); // Update phone
    }

    @Override
    public void start() {
        Thread update = new Thread() {
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
                        tmy += driverMode == 1 ? "Drive Mode: Normal" : driverMode == -1 ? "Driver Mode: Field-Oriented" : "Drive Mode: Null";
                        tmy += "Robot angle: " + getAngle() + "\n";
                        tmy += "Drive: " + drive + "\n";
                        tmy += "Turn: " + turn + "\n";
                        tmy += "Angle: " + angle + "\n";
                        tmy += "Intake: " + (gamepad1.right_trigger) + "\n";
                        tmy += "Outtake: " + (gamepad1.left_trigger) + "\n";
                        tmy += "X acceleration" + ((robot.gyro0.getGravity().xAccel + robot.gyro1.getGravity().xAccel) / 2) + "\n";
                        tmy += "Y acceleration" + ((robot.gyro0.getGravity().yAccel + robot.gyro1.getGravity().yAccel) / 2) + "\n";
                        tmy += "Z acceleration" + ((robot.gyro0.getGravity().zAccel + robot.gyro1.getGravity().zAccel) / 2) + "\n";
                        endTime = elapsedTime.time();
                        timeDifference = endTime - startingTime;
                        tmy += "Cycle Time: " + timeDifference;
                        telemetry.addData("", tmy);
                    } catch (Exception p_exception) {
                        telemetry.addData("Uh oh", "The driver controller was unable to communicate via telemetry.  For help, please seek a better programmer.");
                    }
                    telemetry.update();
                }
            }

            private synchronized double getAngle() {
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
    }

    @Override
    public void loop() {
        startingTime = elapsedTime.time();
        drive = gamepad1.left_stick_y;
        double hangDrive = gamepad2.left_stick_y;
        turn = -gamepad1.right_stick_x;
        angle = -gamepad1.left_stick_x;

        /* Precision vertical drive */
        if (gamepad1.dpad_down || gamepad1.dpad_up) {
            if (gamepad1.left_stick_y != 0) {
                drive = drive * 0.1; // Slow down joystick driving
            } else {
                if (gamepad1.dpad_down) {
                    drive = 0.1; // Slow drive backwards
                } else {
                    drive = -0.1; // Slow drive forwards
                }
            }
        }

        /* Precision sideways drive */
        if (gamepad1.dpad_right || gamepad1.dpad_left) {
            if (gamepad1.right_stick_x != 0) {
                angle = angle * 0.1; // Slow down joystick side movement
            } else {
                if (gamepad1.dpad_left) {
                    angle = 0.1; // Slow leftwards
                } else {
                    angle = -0.1; // Slow rightwards
                }
            }
        }

        /* Precision turn */
        if (gamepad1.left_bumper) {
            turn = 0.1; // Slow left turn
        }
        if (gamepad1.right_bumper) {
            turn = -0.1; // Slow right turn
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

        if((gamepad1.right_trigger - gamepad1.left_trigger) != 0) {
            robot.setInPower(gamepad1.right_trigger - gamepad1.left_trigger); // Set intake power based on the gamepad trigger values
        }
        else {
            if(!autoIntake) {
                robot.setInPower(0);
            }
            else {
                try {
                    if(robot.intake.getCurrentPosition() % 288 != 0 && !robot.intake.isBusy() && autoIntake) {
                        if(robot.intake.getCurrentPosition() % 288 < 144) {
                            robot.setInPos((robot.intake.getCurrentPosition() - ((robot.intake.getCurrentPosition() % 288))), (-1)); // Ensure straight tubing by setting intake position to nearest multiple of 1120
                        }
                        else {
                            robot.setInPos((robot.intake.getCurrentPosition() - ((robot.intake.getCurrentPosition() % 288)) + 288), (1));
                        }
                    }
                } catch (Exception p_exception) {
                    telemetry.addData("Note: ", "I hope you weren't planning on using intake, since it's not really working.");
                }
                try {
                    if (robot.intake.getCurrentPosition() % 288 == 0) {
                        robot.intake.setPower(0);
                    }
                } catch (Exception p_exception) {
                    telemetry.addData("Note: ", "Intake null, good luck");
                }
            }
        }

        if (gamepad2.right_bumper) {
            robot.setKickPower(kickReadyPos); // Ready
        }

        if (gamepad2.left_bumper) {
            robot.setKickPower(kickOutPos); // Eject
        }

        /*
        if (gamepad2.b) {
            bpressedtwo = true;
        }

        if (bpressedtwo && !gamepad2.b) { // Toggle sorting system
            inReady = Math.abs(inReady - 1); // If inReady = 0, set it to 1; otherwise, set it to 0
            forcedOn = false;
            bpressedtwo = false;
        }

        if (gamepad2.a) {
            apressedtwo = true;
        }

        if (apressedtwo && !gamepad2.a) { // Force sorting system on until turned off manually
            forcedOn = true;
            bpressedtwo = false;
        }

        if (inReady == 1 && robot.frontDistanceSensor != null && robot.backDistanceSensor != null) {
            robot.setKickPower(kickReadyPos); // Move kicker out of the way
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
                robot.setKickPower(kickOutPos); // Kick mineral out of container
                frontCube = 0; // the front cube should be not there
            }
        }
        */

        /* Toggle intake auto-straighten */
        if (gamepad1.b && !gamepad1.a) {
            bpressed = true;
        }

        if (bpressed && !gamepad1.b) {
            autoIntake = !autoIntake;
            bpressed = false;
        }

        if (gamepad1.y) {
            ypressed = true;
        }

        if (ypressed && !gamepad1.y) {
            ypressed = false;
            driverMode *= -1;
        }

        /* Reset encoders */
        if (gamepad1.a && !gamepad1.start) {
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

        if (gamepad2.left_stick_y != 0) {
            robot.setKickPower(kickReadyPos);
        }

        if (gamepad2.dpad_down || gamepad2.dpad_up) {
            if (gamepad2.left_stick_y != 0) {
                hangDrive = hangDrive * 0.25; // Slow down joystick hanging
            } else {
                if (gamepad2.dpad_down) {
                    hangDrive = 0.25; // Slow drive hanging
                } else {
                    hangDrive = -0.25; // Slow drive hanging
                }
            }
        }
        robot.setHangPower(hangDrive); // Move container based on gamepad positions
        robot.setExtendPower((Math.abs(gamepad2.right_stick_x) < 0.05 ? gamepad2.dpad_right ? 0.25 : gamepad2.dpad_left ? -0.25 : gamepad1.right_stick_button ? 1 : gamepad1.left_stick_button ? -1 : 0 : gamepad2.dpad_right || gamepad2.dpad_left ? gamepad2.right_stick_x * 0.25 : gamepad2.right_stick_x));
        robot.flipBox(gamepad1.b || gamepad2.right_trigger > 0.01 ? gamepad2.right_trigger > 0.01 ? robot.box.getPosition() + (gamepad2.right_trigger / 10) : robot.box.getPosition() + 0.05 : gamepad1.x || gamepad2.left_trigger > 0.01 ? gamepad2.left_trigger > 0.01 ? -(gamepad2.left_trigger / 10) + robot.box.getPosition() : robot.box.getPosition() - 0.05 : robot.box.getPosition());
    }

    @Override
    public void stop() { // Run when "STOP" pressed
        doTelemetry = false;
        robot.wheelBrake();
        robot.hangBrake();
        robot.setKickPower(kickReadyPos); // Move kick servo to "intake ready" position
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

} // End of class