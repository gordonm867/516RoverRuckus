/* package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Practices.GOFGoldDetect;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Autonomous(name="GOFAutoTestOne", group="GOF")
@Disabled
public class GOFAutonomousGoldDepot extends OpMode {

    /* Declare OpMode members */
/*
    private double xOffsetRange = 40;
    private GOFGoldDetect goldAlign;
    GOFHardware robot = new GOFHardware(); // Use the GOFHardware class
    private ElapsedTime elapsedTime = new ElapsedTime();
    private double drive;
    private double turn;
    private double angle;
    private double unalignedtime;
    private double time;


    @Override
    public void init() {
        goldAlign = new GOFGoldDetect();
        goldAlign.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldAlign.setShowCountours(false);
        // start the vision system
        goldAlign.enable();
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        /* Descend script HERE */ /*
        robot.setKickPower(0.322); // Move kicker out of the way
        robot.setDoorPower(0.186); // Open intake

    }

    @Override
    public void loop() {
        time = Double.parseDouble(elapsedTime.toString().substring(elapsedTime.toString().indexOf('s')));
        goldAlign.setShowCountours(true);
        // get a list of contours from the vision system
        List<MatOfPoint> contours = goldAlign.getContours();
        if (contours.size() > 0) {
            Rect[] rects = new Rect[contours.size()];
            double[] areas = new double[rects.length];
            double[] xPos = new double[rects.length];
            double[] yPos = new double[rects.length];
            for (int i = 0; i < rects.length; i++) {
                try {
                    Rect boundingRect = Imgproc.boundingRect(contours.get(i));
                    rects[i] = boundingRect;
                    areas[i] = boundingRect.area();
                    xPos[i] = (boundingRect.x + boundingRect.width) / 2;
                    yPos[i] = (boundingRect.y + boundingRect.height) / 2;
                } catch (Exception ArrayIndexOutOfBoundsException) {
                    break;
                }
                int maxIndex = maxArea(areas);
                if (maxIndex < xPos.length && maxIndex < yPos.length) {
                    if (xPos[maxIndex] + xOffsetRange > 120 && xPos[maxIndex] - xOffsetRange < 120) {
                        telemetry.addData("Aligned: ", "True!!!");
                        turn = 0;
                        unalignedtime = Double.parseDouble(elapsedTime.toString().substring(elapsedTime.toString().indexOf('s')));
                    }
                    else {
                        telemetry.addData("Aligned: ", "False :(");
                        telemetry.addData("xPos: ", "" + xPos[maxIndex]);
                        telemetry.addData("yPos: ", "" + yPos[maxIndex]);
                        if (xPos[maxIndex] + xOffsetRange < 120 && unalignedtime + 1 < time) {
                            turn = -0.25;
                        }
                        else if (xPos[maxIndex] - xOffsetRange > 120 && unalignedtime + 1 < time) {
                            turn = 0.25;
                        }
                        else {
                            turn = 0;
                        }
                    }
                }
                telemetry.update();
            }
        }
        if((robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 0.75)) {
            telemetry.addData("Status: ", "Mineral found");
            telemetry.update();
            robot.rrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lrWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lfWheel.setTargetPosition(1120);
            robot.lrWheel.setTargetPosition(1120);
            robot.rrWheel.setTargetPosition(1120);
            robot.rfWheel.setTargetPosition(1120);
            robot.setDrivePower(1, 1, 1, 1);
            while(robot.rrWheel.isBusy()) {} // Wait until the wheels are at their target positions
            robot.setDrivePower(0, 0, 0, 0);
            robot.setKickPower(0.766);
            robot.setDoorPower(0.749);
            robot.rrWheel.setTargetPosition((int)((1/2) * (43.18 * Math.PI) * 1120 * (1 / (10 * Math.PI))));
            robot.rfWheel.setTargetPosition((int)((1/2) * (43.18 * Math.PI) * 1120 * (1 / (10 * Math.PI))));
            robot.setDrivePower(1, 1, 1, 1);
            while(robot.rrWheel.isBusy()) {} // Wait until the wheels are at their target positions
            robot.rrWheel.setTargetPosition((int)(1120 * (23.32 / Math.PI)));
            robot.rfWheel.setTargetPosition((int)(1120 * (23.32 / Math.PI)));
            robot.lrWheel.setTargetPosition(((int)(1120 * (23.32 / Math.PI))));
            robot.lfWheel.setTargetPosition(((int)(1120 * (23.32 / Math.PI))));
            robot.setDrivePower(1, 1, 1, 1);
            while(robot.rrWheel.isBusy()) {} // Wait until the wheels are at their target positions
            robot.setDrivePower(0, 0, 0, 0);
            stop();
        }

        robot.setDrivePower(drive + turn - angle, drive + turn + angle, drive - turn + angle, drive - turn - angle); // Set motors to values
        robot.setInPower(0.75);

    }


    public void stop() {
        // stop the vision system
        goldAlign.disable();
        robot.setDrivePower(0, 0, 0, 0);

    }

    public int maxArea(double areas[]) {
        if (areas.length > 0) {
            double max = Double.MIN_VALUE;
            int index = 0;
            for (int i = 0; i < areas.length; i++) {
                double token = areas[i];
                if (token > max) {
                    max = token;
                    index = i;
                }
            }
            telemetry.addData("Index: ", "" + index);
            return index;
        } else {
            telemetry.addData("Index: ", "Error");
            return 0;
        }
    }
} */