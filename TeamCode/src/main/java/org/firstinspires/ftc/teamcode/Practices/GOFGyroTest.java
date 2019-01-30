package org.firstinspires.ftc.teamcode.Practices;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.teamcode.GOFHardware;

@TeleOp(name="GyroTest",group="GOFTests")
@Disabled
public class GOFGyroTest extends LinearOpMode {

    BNO055IMU gyro0;
    BNO055IMU gyro1;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    OpModeManagerImpl manager = (OpModeManagerImpl)this.internalOpModeServices;
    GOFHardware robot = GOFHardware.getInstance(manager);
    ElapsedTime time = new ElapsedTime();
    Orientation g0angles;
    private double angleOffset = 0.1;

    public void runOpMode() {
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro0 = hardwareMap.get(BNO055IMU.class, "g0");
        gyro1 = hardwareMap.get(BNO055IMU.class, "g1");
        gyro0.initialize(parameters);
        gyro1.initialize(parameters);

        while((!gyro0.isGyroCalibrated() && opModeIsActive())) {}
        while((!gyro1.isGyroCalibrated() && opModeIsActive())) {}

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            g0angles = gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            adjustAngle();
            sleep(5000);
        }
    }

    private void adjustAngle() {
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Orientation g0angles = gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation g1angles = gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while (opModeIsActive() && Math.abs((g0angles.firstAngle + g1angles.firstAngle) / 2.0) % 45 > angleOffset) {
            g0angles = gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            g1angles = gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2);
            if (robotAngle % 45 > 22.5) {
                robot.setDrivePower(0.1, 0.1, -0.1, -0.1);
                robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2);
                if (Math.abs(robotAngle) % 45 < angleOffset) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
            else {
                robot.setDrivePower(-0.1, -0.1, 0.1, 0.1);
                robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2);
                if (Math.abs(robotAngle) % 45 < angleOffset) {
                    robot.setDrivePower(0, 0, 0, 0);
                    break;
                }
            }
        }
    }
}
