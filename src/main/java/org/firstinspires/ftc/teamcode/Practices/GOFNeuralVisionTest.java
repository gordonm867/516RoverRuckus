package org.firstinspires.ftc.teamcode.Practices;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name="GOFNeuralVisionTest",group="GOFTests")
@Disabled

public class GOFNeuralVisionTest extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold";
    private static final String LABEL_SILVER_MINERAL = "Silver";
    private static final String VUFORIA_KEY = "AWVhzQD/////AAABmWz790KTAURpmjOzox2azmML6FgjPO5DBf5SHQLIKvCsslmH9wp8b5zkCGfES8tt+8xslwaK7sd2h5H1jwmix26x+Eg5j60l00SlNiJMDAp5IOMWvhdJGZ8jJ8wFHCNkwERQG57JnrOXVSFDlc1sfum3oH68fEd8RrA570Y+WQda1fP8hYdZtbgG+ZDVG+9XyoDrToYU3FYl3WM1iUphAbHJz1BMFFnWJdbZzOicvqah/RwXqtxRDNlem3JdT4W95kCY5bckg92oaFIBk9n01Gzg8w5mFTReYMVI3Fne72/KpPRPJwblO0W9OI3o7djg+iPjxkKOeHUWW+tmi6r3LRaKTrIUfLfazRu0QwLA8Bgw";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector detector;

    public GOFNeuralVisionTest() {}

    public void runOpMode() {
        telemetry.addData("Status: ", "Initialization begun");
        telemetry.update(); // Update phones

        vuforiaInit(); // Initialize Vuforia

        detectInit(); // Initialize TensorFlow detection system

        telemetry.addData("Status: ", "Initialized");
        telemetry.update(); // Update phones

        waitForStart();

        if(!(detector == null)) {
            detector.activate(); // Begin detection
        }

        while (opModeIsActive()) {
            telemetry.addData("Status:", "Started");

            if (detector != null) { // The detector will be null if it's not supported on the device, which shouldn't be a concern, but this helps guarantee no crashes
                List<Recognition> updatedRecognitions = detector.getUpdatedRecognitions(); // ArrayList of detected objects
                if (updatedRecognitions != null) { // List will be null if no objects are detected
                    telemetry.addData("Object Detected: ", updatedRecognitions.size()); // Tell the phones the number of detected objects
                    updateTelemetry(telemetry);
                    while (!(telemetry.update())) {}
                    if (updatedRecognitions.size() == 3) { // If there are three detected objects (the minerals)
                        double goldMineralX = -987654; // Sets all of the object x positions to number well outside actual x range; if they still equal this number, they couldn't have been changed later in the code
                        double silverMineral1X = -987654;
                        double silverMineral2X = -987654;
                        for (Recognition recognition : updatedRecognitions) { // For each item in the list of recognized items
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) { // If the item is gold....
                                goldMineralX = (int)(recognition.getLeft()); // Set the gold x position to its x position
                            }
                            else if (silverMineral1X == -987654) { // If the item is silver and no silver has been assigned an x position yet....
                                silverMineral1X = (int)(recognition.getLeft()); // Set the first silver x position to its x position
                            }
                            else { // If the item is silver and another silver has been found
                                silverMineral2X = (int)(recognition.getLeft()); // Set the second silver x position to its x position
                            }
                        }
                        if (goldMineralX != -987654 && silverMineral1X != -987654 && silverMineral2X != -987654) { // If all of the minerals have new x positions....
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) { // If gold has the lowest x position
                                telemetry.addData("Gold Mineral Position: ", "Left"); // Tell phones it's on the left
                            }
                            else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) { // If gold has the highest x position
                                telemetry.addData("Gold Mineral Position: ", "Right"); // Tell phones it's on the right
                            }
                            else { // Otherwise....
                                telemetry.addData("Gold Mineral Position: ", "Center"); // Tell phones it's in the center
                            }
                        }
                    }
                    updateTelemetry(telemetry);
                    while(!telemetry.update()) {};
                }
            }
        }
    }

    /* public Bitmap getImage(VuforiaLocalizer vuforia) {
        vuforia.setFrameQueueCapacity(5);
        VuforiaLocalizer.CloseableFrame frame;
        try {
            frame = vuforia.getFrameQueue().take();
            for (int i = 0; i < frame.getNumImages(); i++) {
                Image image = frame.getImage(i);
                if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                    int imageWidth = image.getHeight(); // For some reason the height has to be greater than the width
                    int imageHeight = image.getWidth(); // So reversing the two changes the orientation to guarantee this
                    return Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
                }
            }
        }
        catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        return null;
    } */

    private void vuforiaInit() { // Initialize Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void detectInit() { // Initialize TensorFlow detector
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        detector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        detector.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
