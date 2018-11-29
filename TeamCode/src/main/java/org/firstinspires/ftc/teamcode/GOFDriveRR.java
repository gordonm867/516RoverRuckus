package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class GOFDriveRR extends MecanumDrive {

    public GOFDriveRR() {
        super(16);
    }

    GOFHardware robot = GOFHardware.getInstance();

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setDrivePower(v1, v, v2, v3);
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        List<DcMotor> motors = Arrays.asList(robot.lfWheel, robot.lrWheel, robot.rrWheel, robot.rfWheel);
        for (DcMotor motor : motors) {
            wheelPositions.add(4 * Math.PI * motor.getCurrentPosition() / -1120);
        }
        return wheelPositions;
    }

    @Override
    public double getExternalHeading() {
        Orientation g0angles = null;
        Orientation g1angles = null;
        if (robot.gyro0 != null) {
            g0angles = robot.gyro0.getAngularOrientation(); // Get z axis angle from first gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
        }
        if (robot.gyro1 != null) {
            g1angles = robot.gyro1.getAngularOrientation(); // Get z axis angle from second gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
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
        return robotAngle;
    }
}
