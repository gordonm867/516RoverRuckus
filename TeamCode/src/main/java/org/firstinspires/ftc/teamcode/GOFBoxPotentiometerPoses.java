package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

// @SuppressWarnings({"WeakerAccess", "SpellCheckingInspection", "EmptyCatchBlock", "StatementWithEmptyBody", "SameParameterValue"})
@TeleOp(name="GOFBoxTuner", group="GOF")
public class GOFBoxPotentiometerPoses extends OpMode {

    private             boolean             aPressed            = false;
    private             boolean             oaPressed           = false;
    private             boolean             xPressed            = false;
    private             boolean             bPressed            = false;
    private             boolean             yPressed            = false;
    private             boolean             daPressed           = false;
    private             boolean             dxPressed           = false;
    private             boolean             dyPressed           = false;
    private             boolean             dbPressed           = false;
    private             boolean             gotThere            = false;
    private             boolean             useNeg              = false;
    private             boolean             waitingForClick     = true;
    private             boolean             bumperPressed       = false;
    private             boolean             servoMove           = false;
    private             boolean             autoScored          = false;

    private volatile    double              boxPos              = 90;
    private             double              triggerPressed      = 0;
    private             double              lastIntake          = 0;
    private             double              integral            = 0;
    private             double              lastError           = 0;
    private             double              dump                = 75;
    private             double              intake              = 170;
    private             double              neutral             = 90;
    private             double              offset              = 5;
    private             double              Kp                  = 0.05;
    private             double              Ki                  = 0.005;
    private             double              Kd                  = 0.025;

    private             ElapsedTime         elapsedTime         = new ElapsedTime();
    private             ElapsedTime         hangTime            = new ElapsedTime();
    private             ElapsedTime         trigTime            = new ElapsedTime();
    private volatile    ElapsedTime         threadTime          = new ElapsedTime();
    private             ElapsedTime         flipTime            = new ElapsedTime();

    public              GOFHardware         robot               = GOFHardware.getInstance(); // Use the GOFHardware class

    private             int                 iterations          = 0;

    @Override
    public void init() {
        msStuckDetectInit = 10000; // Allow gyros to calibrate
        robot.init(hardwareMap);
        // robot.setKickPower(kickReadyPos);
        robot.teamFlag.setPosition(0.5);
        telemetry.addData("Status", "Initialized"); // Update phone
    }

    @Override
    public void init_loop() {
        checkBox();
        iterations++;
    }

    @Override
    public void loop() {
        telemetry.addData("Run time", elapsedTime.toString());
        telemetry.addData("Kp", ((int)(1000 * Kp) / 1000.0));
        telemetry.addData("Ki", ((int)(1000 * Ki) / 1000.0));
        telemetry.addData("Kd", ((int)(1000 * Kd) / 1000.0));
        telemetry.addData("Box power", robot.box.getPower());

        if(gamepad1.a && !oaPressed) {
            oaPressed = true;
            Kp += 0.005;
        }

        if(oaPressed && !gamepad1.a) {
            oaPressed = false;
        }

        if(gamepad1.x && !xPressed) {
            xPressed = true;
            Ki += 0.005;
        }

        if(xPressed && !gamepad1.x) {
            xPressed = false;
        }

        if(gamepad1.b && !bPressed) {
            bPressed = true;
            Kd += 0.005;
        }

        if(bPressed && !gamepad1.b) {
            bPressed = false;
        }

        if(gamepad1.y && !yPressed) {
            yPressed = true;
            offset += 1;
        }

        if(gamepad1.dpad_up && !daPressed) {
            daPressed = true;
            Kp -= 0.005;
        }

        if(!gamepad1.dpad_up && daPressed) {
            daPressed = false;
        }

        if(gamepad1.dpad_down && !dbPressed) {
            dbPressed = true;
            Ki = 0;
        }

        if(!gamepad1.dpad_down && dbPressed) {
            dbPressed = false;
        }

        if(gamepad1.dpad_left && !dxPressed) {
            dxPressed = true;
            Kd = 0;
        }

        if(!gamepad1.dpad_left && dxPressed) {
            dxPressed = false;
        }

        if(gamepad1.dpad_right && !dyPressed) {
            dyPressed = true;
            offset -= 1;
        }

        if(!gamepad1.dpad_right && dyPressed) {
            dyPressed = false;
        }

        if(gamepad1.left_trigger != 0) {
            triggerPressed = gamepad1.left_trigger * (useNeg ? -1 : 1);
            trigTime.reset();
        }
        if(gamepad1.left_trigger == 0 && triggerPressed != 0) {
            triggerPressed = 0;
            if(!useNeg) {
                waitingForClick = true;
            }
            else {
                useNeg = false;
            }
        }
        if(waitingForClick && trigTime.time() > 0.1) {
            waitingForClick = false;
        }
        if(waitingForClick && gamepad1.left_trigger > 0.05) {
            useNeg = true;
        }

        if((triggerPressed != 0 || gamepad2.dpad_up || gamepad2.dpad_down)) {
            robot.setInPower((gamepad2.dpad_up ? 1 : 0) + triggerPressed - (gamepad2.dpad_down ? 1 : 0) + ((servoMove || autoScored || (Math.abs(gamepad2.right_stick_x) < 0.05 ? gamepad2.dpad_right ? 0.25 : gamepad2.dpad_left ? -0.25 : 0 : gamepad2.right_stick_x) != robot.extend.getPower()) ? 0.25 : 0)); // Set intake power based on the gamepad trigger values
            lastIntake = (robot.intake.getCurrentPosition() - lastIntake);
            lastIntake /= (lastIntake == 0 ? 1 : Math.abs(lastIntake));
        }
        else {
            robot.setInPower(0);
        }

        if(!servoMove) {
            robot.setExtendPower((Math.abs(gamepad2.right_stick_x) < 0.05 ? gamepad2.dpad_right ? 0.25 : gamepad2.dpad_left ? -0.25 : 0 : gamepad2.right_stick_x));
        }

        if(((Math.abs(gamepad2.right_stick_x) < 0.05 ? gamepad2.dpad_right ? 0.25 : gamepad2.dpad_left ? -0.25 : 0 : gamepad2.right_stick_x)) != 0) {
            servoMove = false;
        }

        if(gamepad2.left_trigger != 0) {
            flipBox(neutral); // Neutral
        }

        if(gamepad2.right_trigger != 0) {
            flipBox(intake); // Intake
        }

        if(gamepad2.right_bumper && !bumperPressed) {
            flipBox(dump); // Dump
            hangTime.reset();
        }

        if(bumperPressed && !(gamepad2.right_bumper || gamepad2.left_bumper)) {
            bumperPressed = false;
        }

        if(gamepad2.a && !gamepad2.start && !aPressed) {
            flipBox(neutral);
            aPressed = true;
            servoMove = true;
            robot.setInPower(0.25);
        }

        if(aPressed && !gamepad1.a) {
            aPressed = false;
        }

        if(servoMove && !(robot.extenderSensor.getVoltage() > 2)) {
            robot.setExtendPower(1);
        }

        if(servoMove && !(robot.bottomSensor.isPressed())) {
            robot.setHangPower(-1);
        }

        if(robot.extenderSensor.getVoltage() > 2 && servoMove) {
            robot.setExtendPower(0);
            if(robot.bottomSensor.isPressed()) {
                robot.setHangPower(0);
                servoMove = false;
                flipBox(dump);
                autoScored = true;
            }
        }

        if(robot.bottomSensor.isPressed() && servoMove) {
            robot.setHangPower(0);
            if(robot.extenderSensor.getVoltage() > 2) {
                servoMove = false;
                robot.extend.setPower(0);
                flipBox(dump);
                autoScored = true;
            }
        }

        if(servoMove && robot.bottomSensor.isPressed() && robot.extenderSensor.getVoltage() > 2) {
            robot.setHangPower(0);
            robot.setExtendPower(0);
            servoMove = false;
            flipBox(dump);
            autoScored = true;
        }

        if(autoScored && (Math.abs(robot.boxPotentiometer.getVoltage() - (3.3 * (boxPos / 180))) >= 0.1)) {
            flipTime.reset();
        }

        if(flipTime.time() >= 0.5 && flipTime.time() <= 3 && autoScored && (Math.abs(robot.boxPotentiometer.getVoltage() - (3.3 * (boxPos / 180))) >= 0.1)) {
            autoScored = false;
            flipBox(neutral);
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

    private void flipBox(final double angle) {
        iterations = 0;
        integral = 0;
        boxPos = angle;
    }

    private void checkBox() {
        robot.box.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double currentAngle = 180 * (robot.boxPotentiometer.getVoltage() / 3.3);
        double error = -(boxPos - currentAngle);
        double derivative = 0;
        if((Math.abs(error) >= offset && !gotThere) || Math.abs(error) >= 2.5 * offset) {
            if(iterations > 1) {
                integral += threadTime.time() * (lastError - error);
                derivative = (error - lastError) / threadTime.time();
                if(Math.abs(derivative) >= 50) {
                    derivative = 0;
                }
                if(Math.abs(integral) >= 50) {
                    integral = 0;
                }
                if(error != 0) {
                    derivative = Math.abs(derivative) * (error / Math.abs(error));
                    integral = Math.abs(integral) * (error / Math.abs(error));
                }
            }
            lastError = error;
            double PIDPower;
            if(boxPos >= 170 && currentAngle >= 170) {
                robot.box.setPower(0);
            }
            else {
                try {
                    PIDPower = (Kp * error) + (Ki * integral) + (Kd * (derivative));
                } catch (Exception p_exception) {
                    PIDPower = (Kp
                            * error);
                }
                if(Math.abs(PIDPower) >= 0.15) {
                    if((currentAngle <= 175 || PIDPower <= 0) && (currentAngle >= 20 || PIDPower >= 0)) {
                        robot.box.setPower(Range.clip(PIDPower, -robot.maxBoxSpeed, robot.maxBoxSpeed));
                        threadTime.reset();
                    }
                    else {
                        robot.box.setPower(0);
                    }
                } else {
                    robot.box.setPower(0);
                }
            }
        }
        else {
            robot.box.setPower(0);
            gotThere = true;
        }
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

} // End of class