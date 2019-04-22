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
@TeleOp(name="GOFTeleOpTest", group="GOF")
public class GOFTeleOpTest extends OpMode {

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
        // robot.setKickPower(kickReadyPos);
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
        if(robot.hangOne != null && robot.hangOne.getPower() > 0.05 && boxPos == dump) {
            flipBox(100);
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

}