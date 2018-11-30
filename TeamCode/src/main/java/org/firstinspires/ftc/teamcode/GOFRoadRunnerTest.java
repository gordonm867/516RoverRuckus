package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.InputStream;

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
        }
        waitForStart();
        followTrajectory("Left");
    }

    private void followTrajectory(String position) {
        MecanumDrive drive = new GOFDriveRR();
        TrajectoryConfig config;
        try {
            InputStream inputStream = AppUtil.getDefContext().getAssets().open("trajectory/GOFAutoCrater" + position + ".yaml");
            ObjectMapper MAPPER = new ObjectMapper(new YAMLFactory());
            config = MAPPER.readValue(inputStream, TrajectoryConfig.class);
        }
        catch(Exception p_exception) {
            config = null;
        }
        Trajectory trajectory = new Trajectory();
        if(config != null) {
            trajectory = config.toTrajectory();
        }
        PIDFCoefficients coeffs = ((DcMotorEx)robot.lfWheel).getPIDFCoefficients(robot.lfWheel.getMode());
        PIDCoefficients translationalCoeffs = new PIDCoefficients(coeffs.p, coeffs.i, coeffs.d);
        MecanumPIDVAFollower follower = new MecanumPIDVAFollower(drive, translationalCoeffs, translationalCoeffs, 1 / 25.0, 1 / 30.0, robot.getBatteryVoltage());
        follower.followTrajectory(trajectory);
        elapsedTime.reset();
        while(opModeIsActive() && follower.isFollowing()) {
            follower.update(drive.getPoseEstimate());
            telemetry.addData("Status", "Following trajectory....");
            telemetry.update();
        }
    }

}

