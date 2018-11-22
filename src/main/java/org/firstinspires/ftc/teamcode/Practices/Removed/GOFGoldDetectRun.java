/*
package org.firstinspires.ftc.teamcode.Practices;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@TeleOp(name="VisionDemo")

public class GOFGoldDetectRun extends OpMode {

    private double xOffsetRange = 40;
    private double unalignedtime;
    private ElapsedTime elapsedTime = new ElapsedTime();

    private GOFGoldDetect goldAlign;

    @Override
    public void init() {
        goldAlign = new GOFGoldDetect();
        goldAlign.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldAlign.setShowCountours(false);
        // start the vision system

        goldAlign.enable();
    }

    @Override
    public void loop() {

        time = Double.parseDouble(elapsedTime.toString().substring(0, elapsedTime.toString().indexOf('s')));
        goldAlign.setShowCountours(true);

        // get a list of contours from the vision system
        List<MatOfPoint> contours = goldAlign.getContours();
        if(contours.size() > 0) {
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
                }
                catch(Exception ArrayIndexOutOfBoundsException) {
                    break;
                }
                int maxIndex = maxArea(areas);
                if(maxIndex < xPos.length && maxIndex < yPos.length); {
                    if(xPos[maxIndex] + xOffsetRange > 120 && xPos[maxIndex] - xOffsetRange < 120) {
                        unalignedtime = Double.parseDouble(elapsedTime.toString().substring(0, elapsedTime.toString().indexOf('s')));
                        telemetry.addData("Aligned: ", "True!!!");
                    }
                    else {
                        if (unalignedtime < time) {
                            try {
                                telemetry.addData("Aligned: ", "False :(");
                                telemetry.addData("xPos: ", "" + xPos[maxIndex]);
                                telemetry.addData("yPos: ", "" + yPos[maxIndex]);
                                telemetry.addData("Unaligned time: ", "" + unalignedtime);
                                telemetry.addData("Time: ", "" + Double.parseDouble(elapsedTime.toString().substring(0, elapsedTime.toString().indexOf('s'))));
                            } catch (Exception p_exception) {
                                telemetry.addData("Notice: ", "An error occurred while parsing telemetry data.");
                            }
                        }
                        else {
                            try {
                                telemetry.addData("Aligned: ", "Wait....");
                                telemetry.addData("Unaligned time: ", "" + unalignedtime);
                                telemetry.addData("Time: ", "" + Double.parseDouble(elapsedTime.toString().substring(0, elapsedTime.toString().indexOf('s'))));
                            }
                            catch (Exception p_exception) {
                                telemetry.addData("Notice: ", "An error occurred while parsing telemetry data.");
                            }
                        }
                    }
                }
                telemetry.update();
            }
        }
    }


    public void stop() {
        // stop the vision system
        goldAlign.disable();
    }

    public int maxArea(double areas[]) {
        if(areas.length > 0) {
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
        }
        else {
            telemetry.addData("Index: ", "Error");
            return 0;
        }
    }


} */