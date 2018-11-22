/* package org.firstinspires.ftc.teamcode.Practices;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/* WILL NOT WORK WITHOUT OPENCV */ /*

public class GOFGoldDetect extends OpenCVPipeline {
    private boolean showContours = true;
    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();

    private List<MatOfPoint> contours = new ArrayList<>();

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }

    public synchronized List<MatOfPoint> getContours() {
        return contours;
    }

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 2);
        Core.inRange(hsv, new Scalar(14, 73, 196), new Scalar(220, 244, 252), thresholded);

        Imgproc.blur(thresholded, thresholded, new Size(40, 40));

        contours = new ArrayList<>();

        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        if (showContours) {
            Imgproc.drawContours(rgba, contours, -1, new Scalar(0, 255, 0), 2, 8);
        }

        return rgba;
    }
} */