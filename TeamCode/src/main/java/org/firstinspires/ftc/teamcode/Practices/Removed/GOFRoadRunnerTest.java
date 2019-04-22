package org.firstinspires.ftc.teamcode.Practices.Removed;

/*
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GOFHardware;

@Autonomous(name="GOFRoadRunnerTest",group="GOFTests")
// @Disabled
public class GOFRoadRunnerTest extends LinearOpMode {

    private ElapsedTime elapsedTime = new ElapsedTime();
    private GOFHardware robot = GOFHardware.getInstance(); // Use the GOFHardware class

    public void runOpMode() {
        robot.init(hardwareMap);
        if (robot.rrWheel != null && robot.rfWheel != null && robot.lfWheel != null && robot.lrWheel != null && robot.hangOne != null) {
            robot.rrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.hangOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hangOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        waitForStart();
        doTrajectory("Left");
    }

    private void doTrajectory(String position) {
        if (position.equalsIgnoreCase("Left")) {
            robot.setInPower(0.5);
            DriveConstraints constraints = new DriveConstraints(22.4, 22.4, 90, 45);
            Trajectory trajectory = new TrajectoryBuilder(new Pose2d(25, 25, 45), constraints)
                    .splineTo(new Pose2d(28.0, 47.0, 110))
                    .splineTo(new Pose2d(-62.0, 57.0, 170.0))
                    .splineTo(new Pose2d(-43.0, 27.0, 180.0))
                    .splineTo(new Pose2d(30.00, 57.00, 210))
                    .build();
            followTrajectory(trajectory);
        } else {
            throw new IllegalArgumentException();
        }
    }

    private void followTrajectory(Trajectory trajectory) {
        MecanumDrive drive = new GOFDriveRR();
        PIDFCoefficients coeffs = ((DcMotorEx) robot.lfWheel).getPIDFCoefficients(robot.lfWheel.getMode());
        PIDCoefficients translationalCoeffs = new PIDCoefficients(coeffs.p, coeffs.i, coeffs.d);
        MecanumPIDVAFollower follower = new MecanumPIDVAFollower(drive, translationalCoeffs, translationalCoeffs, 1 / 22.4, 1 / 22.4, robot.getBatteryVoltage());
        follower.followTrajectory(trajectory);
        elapsedTime.reset();
        while (opModeIsActive() && follower.isFollowing()) {
            follower.update(drive.getPoseEstimate());
            telemetry.addData("Status", "Following trajectory....");
            telemetry.update();
        }
    }
}
*/
