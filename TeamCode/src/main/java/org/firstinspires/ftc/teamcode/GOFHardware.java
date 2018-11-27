package org.firstinspires.ftc.teamcode;

/*
Device Directory:

Motor Controller 0 Port 0 rrWheel           (rr)
Motor Controller 0 Port 1 rfWheel           (rf)
Motor Controller 1 Port 0 hangOne           (h1)
Motor Controller 1 Port 1 intake            (inw)
Motor Controller 1 Port 2 lfWheel           (lf)
Motor Controller 1 Port 3 lrWheel           (lr)

Digital Controller 1 Port 0 topSensor       (tl)
Digital Controller 1 Port 1 bottomSensor    (tb)

Servo Controller 1 Port 5 kicker            (s0)

I2C Controller 0 Port 0 gyro0               (g0)
I2C Controller 1 Port 0 gyro1               (g1)
*/

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GOFHardware {

    public          BNO055IMU        gyro0;
    public          BNO055IMU        gyro1;

    public          boolean          leftFound;
    public          boolean          centerFound;
    public          boolean          rightFound;
    public          boolean          soundError;

    public          ColorSensor      frontColorSensor;
    public          ColorSensor      backColorSensor;

    public          DcMotor          lfWheel;
    public          DcMotor          rfWheel;
    public          DcMotor          lrWheel;
    public          DcMotor          rrWheel;
    public          DcMotor          intake;
    public          DcMotor          hangOne;

    public          DistanceSensor   frontDistanceSensor;
    public          DistanceSensor   backDistanceSensor;

    public          double           maxDriveSpeed           = 0.7;

    private static  GOFHardware     robot                    = null;

    public HardwareMap              hwMap;

    public Orientation              g0angles;
    public Orientation              g1angles;

    public Integer                  rightId;
    public Integer                  centerId;
    public Integer                  leftId;

    public RevTouchSensor           topSensor;
    public RevTouchSensor           bottomSensor;

    public Servo            kicker;

    /* Constructor */
    public static GOFHardware getInstance() {
        if(robot == null) {
            robot = new GOFHardware();
        }
        return robot;
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

   /*
         ---------------------------------------
              MOTORS (Define and Initialize)
         ---------------------------------------
    */

        try { // Left rear wheel
            lrWheel = hwMap.get(DcMotor.class, "lr");
            lrWheel.setDirection(DcMotor.Direction.REVERSE);
            lrWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lrWheel.setPower(0);
            lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) {
            lrWheel = null;
        }

        try { // Left front wheel
            lfWheel = hwMap.get(DcMotor.class, "lf");
            lfWheel.setDirection(DcMotor.Direction.REVERSE);
            lfWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lfWheel.setPower(0);
            lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) {
            lfWheel = null;
        }

        try { // Right rear wheel
            rrWheel = hwMap.get(DcMotor.class, "rr");
            rrWheel.setDirection(DcMotor.Direction.FORWARD);
            rrWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rrWheel.setPower(0);
            rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) {
            rrWheel = null;
        }

        try { // Right front wheel
            rfWheel = hwMap.get(DcMotor.class, "rf");
            rfWheel.setDirection(DcMotor.Direction.FORWARD);
            rfWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rfWheel.setPower(0);
            rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) {
            rfWheel = null;
        }

        try { // Intake
            intake = hwMap.get(DcMotor.class, "in");
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setPower(0);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        catch (Exception p_exception) {
            intake = null;
        }


        try { // Hang
            hangOne = hwMap.get(DcMotor.class, "h1");
            hangOne.setDirection(DcMotor.Direction.FORWARD);
            hangOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hangOne.setPower(0);
            hangOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception p_exception) {
            hangOne = null;
        }

    /*
         ---------------------------------------
              SERVOS (Define and Initialize)
         ---------------------------------------
    */

        try { // Container kicker servo
            kicker = hwMap.get(Servo.class, "s0");
        }
        catch (Exception p_exception) {
            kicker = null;
        }

        /*
         ---------------------------------------
              SENSORS (Define and Initialize)
         ---------------------------------------
    */

        try { // Front container color sensor
            frontColorSensor = hwMap.get(ColorSensor.class, "cd0");
        }
        catch (Exception p_exception) {
            frontColorSensor = null;
        }

        try { // Rear container color sensor
            backColorSensor = hwMap.get(ColorSensor.class, "cd1");
        }
        catch (Exception p_exception) {
            backColorSensor = null;
        }

        try { // Front container distance sensor
            frontDistanceSensor = hwMap.get(DistanceSensor.class, "cd0");
        }
        catch (Exception p_exception) {
            frontDistanceSensor = null;
        }

        try { // Sound files
            rightId = hwMap.appContext.getResources().getIdentifier("right", "raw", hwMap.appContext.getPackageName());
            leftId = hwMap.appContext.getResources().getIdentifier("left", "raw", hwMap.appContext.getPackageName());
            centerId = hwMap.appContext.getResources().getIdentifier("center", "raw", hwMap.appContext.getPackageName());
            if(rightId != 0) {
                rightFound = SoundPlayer.getInstance().preload(hwMap.appContext, rightId);
            }
            if(leftId != 0) {
                leftFound = SoundPlayer.getInstance().preload(hwMap.appContext, leftId);
            }
            if(centerId != 0) {
                centerFound = SoundPlayer.getInstance().preload(hwMap.appContext, centerId);
            }
        }
        catch (Exception p_exception) {
            rightId = null;
            leftId = null;
            centerId = null;
        }

        try { // Rear container distance sensor
            backDistanceSensor = hwMap.get(DistanceSensor.class, "cd1");
        }
        catch (Exception p_exception) {
            backDistanceSensor = null;
        }

        try { // Gyro 0
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            gyro0 = hwMap.get(BNO055IMU.class, "g0");
            gyro0.initialize(parameters);
        }
        catch (Exception p_exception) {
            gyro0 = null;
        }

        try { // Gyro 1
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            gyro1 = hwMap.get(BNO055IMU.class, "g1");
            gyro1.initialize(parameters);
        }
        catch (Exception p_exception) {
            gyro1 = null;
        }

        try { // Upper limit switch
            topSensor = hwMap.get(RevTouchSensor.class, "tl");
        }
        catch (Exception p_exception) {
            topSensor = null;
        }

        try { // Lower limit switch
            bottomSensor = hwMap.get(RevTouchSensor.class, "tb");
        }
        catch (Exception p_exception) {
            bottomSensor = null;
        }
    }

    /*
         ======================

              M E T H O D S

         ======================
    */

    /*

         ----------------------
                 MOTORS
         ----------------------
    */

    public void setDrivePower(double leftBackDrivePower, double leftFrontDrivePower, double rightBackDrivePower, double rightFrontDrivePower) { // Send power to wheels
        if (lrWheel != null) {
            leftBackDrivePower = Range.clip(leftBackDrivePower, -maxDriveSpeed, maxDriveSpeed);
            lrWheel.setPower(leftBackDrivePower);
        }
        if (lfWheel != null) {
            leftFrontDrivePower = Range.clip(leftFrontDrivePower, -maxDriveSpeed, maxDriveSpeed);
            lfWheel.setPower(leftFrontDrivePower);
        }
        if (rrWheel != null) {
            rightBackDrivePower = Range.clip(rightBackDrivePower, -maxDriveSpeed, maxDriveSpeed);
            rrWheel.setPower(rightBackDrivePower);
        }
        if (rfWheel != null) {
            rightFrontDrivePower = Range.clip(rightFrontDrivePower, -maxDriveSpeed, maxDriveSpeed);
            rfWheel.setPower(rightFrontDrivePower);
        }
    }

    public void setHangPower(double hangPower) { // Set hang power
        if(hangOne != null) {
            hangPower = Range.clip(hangPower, -1, 1);
            if(topSensor != null) {
                if(topSensor.isPressed() && hangPower > 0) { // If the top sensor is being pressed but the intended hang power is positive, stop
                    hangPower = 0;
                }
            }
            if(bottomSensor != null) {
                if(bottomSensor.isPressed() && hangPower < 0) { // If the bottom sensor is being pressed but the intended hang power is negative, stop
                    hangPower = 0;
                }
            }
            hangOne.setPower(hangPower);
        }
    }

    public void setInPower(double inPower) { // Set intake power
        if (intake != null) {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            inPower = Range.clip(inPower, -0.7, 0.7);
            intake.setPower(inPower);
        }
    }

    public void setInPos(int inPos, double inPower) { // Set intake position
        if (intake != null) {
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setTargetPosition(inPos);
            intake.setPower(inPower);
        }
    }

    public void setKickPower(double kickerPosition) { // Move kicker servo
        if(kicker != null) {
            kickerPosition = Range.clip(kickerPosition, 0, 1);
            kicker.setPosition(kickerPosition);
        }
    }

    public void playSound(double goldPos) { // Play sound
        soundError = false;
        try {
            if (goldPos == -1) {
                SoundPlayer.getInstance().startPlaying(hwMap.appContext, leftId);
            }
            else if (goldPos == 0) {
                SoundPlayer.getInstance().startPlaying(hwMap.appContext, centerId);
            }
            else if (goldPos == 1) {
                SoundPlayer.getInstance().startPlaying(hwMap.appContext, rightId);
            }
        }
        catch (Exception p_exception) {
            soundError = true;
        }
    }

    public void wheelBrake() { // Stop driving
        setDrivePower(0,0,0,0);
    }

    public void hangBrake() { // Stop hang movement
        setHangPower(0);
    }

    public void gyroInit() { // Re-initialize gyros
        try {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            gyro0 = hwMap.get(BNO055IMU.class, "g0");
            gyro0.initialize(parameters);
        }
        catch (Exception p_exception) {
            gyro0 = null;
        }

        try {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            gyro1 = hwMap.get(BNO055IMU.class, "g1");
            gyro1.initialize(parameters);
        }
        catch (Exception p_exception) {
            gyro1 = null;
        }
    }

} //End of class