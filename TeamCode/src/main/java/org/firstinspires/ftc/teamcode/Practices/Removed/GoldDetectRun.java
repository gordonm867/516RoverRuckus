/*
package org.firstinspires.ftc.teamcode.Practices.Removed;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;


@TeleOp(name="GoldAlign", group="GOFTests")
public class GOFGoldDetectRun extends OpMode {
    private double xOffset = 35;
    private double yOffset = 35;
    private GOFGoldDetect goldVision;
    @Override
    public void init() {
        goldVision = new GOFGoldDetect();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        goldVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldVision.setShowCountours(true);
        // start the vision system
        goldVision.enable();
    }

    @Override
    public void loop() {
        List<MatOfPoint> contours = goldVision.getContours();
        double[] rectSizes = new double[contours.size()];
        double[] xPos = new double[contours.size()];
        double[] yPos = new double[contours.size()];
        try {
            for (int i = 0; i < (contours.size() - 1); i++) {
                Rect boundingRect = Imgproc.boundingRect(contours.get(i));
                rectSizes[i] = (boundingRect.width * boundingRect.height);
                xPos[i] = boundingRect.x;
                yPos[i] = boundingRect.y;
            }
        }
        catch (Exception ArrayIndexOutOfBoundsException) {
            telemetry.addData("Note: ", "Error with pre-developed code.");
        }

        int max = 0;
        telemetry.addData("Note: ", "Set max");
        telemetry.addData("rectSizes: ", "" + rectSizes.length);
        telemetry.addData("xPos: ", "" + xPos.length);
        telemetry.addData("yPos: ", "" + yPos.length);
        for (int i = 1; i < (rectSizes.length - 1); i++) {
            try {
                if (rectSizes[i] > rectSizes[max]) {
                    max = i;
                    telemetry.addData("Max: ", "" + max);
                    telemetry.update();
                }
            }
            catch (Exception p_exception) {
                telemetry.addData("Note: ", "Caught exception rectSizes");
                telemetry.update();
                break;
            }
        }
        telemetry.addData("Note: ", "Found best rectangle");
        telemetry.update();

        try {
            telemetry.addData("xPos for max: ", "" + xPos.length);
            telemetry.addData("yPos for max: ", "" + yPos.length);
            telemetry.addData("Max for error: ", "" + max);
            if (xPos[max] > -xOffset && xPos[max] < xOffset && yPos[max] > -yOffset && yPos[max] < yOffset) {
                telemetry.addData("Aligned: ", "True");
            } else {
                telemetry.addData("Aligned: ", "False");
            }
        }
        catch (Exception p_exception) {
            telemetry.addData("Note: ", "Caught exception xPos or yPos");
            telemetry.update();
        }
    }

    public void stop() {
        goldVision.disable();
    }
} */