package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import android.util.Base64;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Iterator;
import java.util.List;

@TeleOp(name="GOFAutoCreatorTest",group="GOFTests")
@Disabled
public class AutoCreatorTest extends LinearOpMode {
    private                 GOFTeleOp           driveControl            = GOFTeleOp.getInstance();

    private static final    String              TFOD_MODEL_ASSET        = "RoverRuckus.tflite";
    private static final    String              LABEL_GOLD_MINERAL      = "Gold Mineral";
    private static final    String              LABEL_SILVER_MINERAL    = "Silver Mineral";
    private static final    String              VUFORIA_KEY             = "AWVhzQD/////AAABmWz790KTAURpmjOzox2azmML6FgjPO5DBf5SHQLIKvCsslmH9wp8b5zkCGfES8tt+8xslwaK7sd2h5H1jwmix26x+Eg5j60l00SlNiJMDAp5IOMWvhdJGZ8jJ8wFHCNkwERQG57JnrOXVSFDlc1sfum3oH68fEd8RrA570Y+WQda1fP8hYdZtbgG+ZDVG+9XyoDrToYU3FYl3WM1iUphAbHJz1BMFFnWJdbZzOicvqah/RwXqtxRDNlem3JdT4W95kCY5bckg92oaFIBk9n01Gzg8w5mFTReYMVI3Fne72/KpPRPJwblO0W9OI3o7djg+iPjxkKOeHUWW+tmi6r3LRaKTrIUfLfazRu0QwLA8Bgw";

    private                 GOFVuforiaLocalizer vuforia;
    private                 TFObjectDetector    detector;

    private                 ElapsedTime         elapsedTime             = new ElapsedTime();

    private                 boolean             remove;
    private                 boolean             crater                  = false;
    private                 boolean             doubleSample            = true;
    private                 boolean             yPressed                = false;

    private                 double              startTime               = elapsedTime.time();

    private                 int                 goldPos                 = -2;

    public void runOpMode() {
        driveControl.init();
        Writer fileWriter;
        while(!gamepad1.x) {
            telemetry.addData("Double Sampling is", (doubleSample ? "ON" : "OFF") + " - Press \"Y\" to change and \"X\" to finalize (on gamepad1)");
            telemetry.update();
            if(gamepad1.y && !yPressed) {
                doubleSample = !doubleSample;
                yPressed = true;
            }
            else {
                yPressed = false;
            }
        }
        while(gamepad1.x) {}
        while(!gamepad1.x) {
            telemetry.addData("This autonomous path begins in front of the ", (crater ? "CRATER" : "DEPOT") + " - Press \"Y\" to change and \"X\" to finalize (on gamepad1)");
            telemetry.update();
            if(gamepad1.y && !yPressed) {
                crater = !crater;
                yPressed = true;
            }
            else {
                yPressed = false;
            }
        }
        double goldPos = detectGold();
        try {
            File file = new File(Environment.getExternalStorageDirectory().getPath() + File.separator + (crater ? (doubleSample ? "GOFCratAutoDS" : "GOFCratAutoSS") : "GOFDepAuto") + ((goldPos == 0) ? "Center" : (goldPos == 1) ? "Right" : "Left") + ".txt");
            if(!file.exists()) {
                file.mkdirs();
                file.createNewFile();
            }
            FileWriter writer = new FileWriter(Environment.getExternalStorageDirectory().getPath() + File.separator + "GOFDepAuto" + ((goldPos == 0) ? "Center" : (goldPos == 1) ? "Right" : "Left") + ".txt");
            fileWriter = new BufferedWriter(writer);
        } catch (IOException e) {
            throw new RuntimeException("Cannot write to file", e);
        }

        waitForStart();
        driveControl.start();
        while(opModeIsActive()) {
            driveControl.loop();
            try {
                fileWriter.write(stringify(gamepad1) + "\n");
                fileWriter.write(stringify(gamepad2) + "\n");
            }
            catch(Exception noWorkException) {
                giveUp();
            }
        }
        try {
            fileWriter.close();
        }
        catch(Exception e_exception) {
            telemetry.addData("Note", "This file can't be saved, so you just wasted quite a bit of time.");
        }
        driveControl.stop();
    }

    private String stringify(Gamepad gamepad) throws RobotCoreException {
        return Base64.encodeToString(gamepad.toByteArray(), Base64.NO_WRAP);
    }

    private void giveUp() {
        telemetry.addData("Message from programmer", "I give up. :(");
        telemetry.update();
        while(opModeIsActive()) {
            sleep(500);
        }
    }

    private int detectGold() {
        vuforiaInit();
        detectInit();
        while (!isStopRequested() && !isStarted()) {
            if (detector != null) { // The detector will be null if it's not supported on the device, which shouldn't be a concern, but this helps guarantee no crashes
                Recognition[] sampleMinerals = new Recognition[2];
                List<Recognition> updatedRecognitions = detector.getUpdatedRecognitions(); // ArrayList of detected objects
                if (updatedRecognitions != null) { // List will be null if no objects are detected
                    Iterator<Recognition> updatedRecognitionsItr = updatedRecognitions.iterator();
                    telemetry.addData("Object Detected", updatedRecognitions.size()); // Tell the phones the number of detected objects
                    telemetry.update();
                    while(updatedRecognitionsItr.hasNext()) {
                        telemetry.addData("Status", "Filtering out double-detections....");
                        telemetry.update();
                        Recognition recognition = updatedRecognitionsItr.next();
                        if(updatedRecognitions.size() > 2) {
                            for(Recognition recognitionNested : updatedRecognitions) {
                                if((recognitionNested.getTop() + 10 > recognition.getTop()) && (recognitionNested.getTop() - 10 < recognition.getTop()) && (recognitionNested.getLeft() + 10 > recognition.getLeft() && recognitionNested.getLeft() - 10 < recognition.getLeft())) {
                                    if(recognitionNested != recognition) {
                                        remove = true;
                                    }
                                }
                            }
                            if(remove) {
                                updatedRecognitionsItr.remove();
                                remove = false;
                            }
                            if(updatedRecognitions.size() > 2) {
                                telemetry.addData("Status", "Filtering out Depot.... (this is really bad since you're not actually running Depot auto, by the way)");
                                telemetry.update();
                                Recognition min1 = null;
                                Recognition min2 = null;
                                double minRecY = Double.MAX_VALUE;
                                for(Recognition minFind : updatedRecognitions) {
                                    if(minFind.getTop() < minRecY) {
                                        minRecY = minFind.getTop();
                                        min1 = minFind;
                                    }
                                }
                                minRecY = Double.MAX_VALUE;
                                for(Recognition minFind : updatedRecognitions) {
                                    if(minFind.getTop() < minRecY && minFind != min1) {
                                        minRecY = minFind.getTop();
                                        min2 = minFind;
                                    }
                                }
                                sampleMinerals[0] = min1;
                                sampleMinerals[1] = min2;
                                updatedRecognitionsItr = updatedRecognitions.iterator();
                                while(updatedRecognitionsItr.hasNext()) {
                                    recognition = updatedRecognitionsItr.next();
                                    if(recognition != min1 && recognition != min2) {
                                        updatedRecognitionsItr.remove();
                                    }
                                }
                            }
                        }
                    }
                    if(updatedRecognitions.size() == 3) { // If there are three detected objects (the minerals)
                        double goldMineralX = -987654; // Sets all of the object x positions to number well outside actual x range; if they still equal this number, they couldn't have been changed later in the code
                        double silverMineral1X = -987654;
                        double silverMineral2X = -987654;
                        for(Recognition recognition : updatedRecognitions) { // For each item in the list of recognized items
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
                                telemetry.addData("Gold Mineral Position?", "Left, x pos " + goldMineralX); // Tell phones it might be on the left
                                goldPos = -1;
                            }
                            else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) { // If gold has the highest x position
                                telemetry.addData("Gold Mineral Position?", "Right, x pos " + goldMineralX); // Tell phones it might be on the right
                                telemetry.update();
                                goldPos = 1;
                            }
                            else { // Otherwise....
                                telemetry.addData("Gold Mineral Position?", "Center, x pos " + goldMineralX);
                                telemetry.update();
                                goldPos = 0;
                            }
                            telemetry.update();
                        }
                    }
                    else if (updatedRecognitions.size() == 2) { // If only left two are visible
                        double goldMineralX = -987654; // Sets all of the object x positions to number well outside actual x range; if they still equal this number, they couldn't have been changed later in the code
                        double silverMineral1X = -987654;
                        double silverMineral2X = -987654;
                        for (Recognition recognition : updatedRecognitions) { // For each item in the list of recognized items
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) { // If the item is gold....
                                goldMineralX = (int)(recognition.getLeft()); // Set the gold x position to its x position (note that the phone's orientation reverses axes)
                            } else if (silverMineral1X == -987654) { // If the item is silver and no silver has been assigned an x position yet....
                                silverMineral1X = (int)(recognition.getLeft()); // Set the first silver x position to its x position
                            } else {
                                silverMineral2X = (int)(recognition.getLeft());
                            }
                            if (silverMineral2X == -987654 && goldMineralX != -987654 && silverMineral1X != -987654) {
                                if(goldMineralX > silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Center, x pos " + goldMineralX + "; silver x pos" + silverMineral1X);
                                    telemetry.update();
                                    goldPos = 0;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Left, x pos " + goldMineralX + "; silver x pos" + silverMineral1X);
                                    telemetry.update();
                                    goldPos = -1;
                                }
                            } else if (silverMineral2X != -987654) {
                                telemetry.addData("Gold Mineral Position", "Right, x pos " + goldMineralX);
                                telemetry.update();
                                goldPos = 1;
                            }
                        }
                    }
                    else {
                        try {
                            double goldMineralX = -987654; // Sets all of the object x positions to number well outside actual x range; if they still equal this number, they couldn't have been changed later in the code
                            double silverMineral1X = -987654;
                            double silverMineral2X = -987654;
                            for (Recognition recognition : sampleMinerals) { // For each item in the list of recognized items
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) { // If the item is gold....
                                    goldMineralX = (int) (recognition.getLeft()); // Set the gold x position to its x position (note that the phone's orientation reverses axes)
                                } else if (silverMineral1X == -987654) { // If the item is silver and no silver has been assigned an x position yet....
                                    silverMineral1X = (int)(recognition.getLeft()); // Set the first silver x position to its x position
                                } else {
                                    silverMineral2X = (int)(recognition.getLeft());
                                }
                                if (silverMineral2X == -987654 && goldMineralX != -987654 && silverMineral1X != -987654) {
                                    if (goldMineralX > silverMineral1X) {
                                        telemetry.addData("Gold Mineral Position", "Center, x pos " + goldMineralX + "; silver x pos" + silverMineral1X);
                                        telemetry.update();
                                        goldPos = 0;
                                    } else {
                                        telemetry.addData("Gold Mineral Position", "Left, x pos " + goldMineralX + "; silver x pos" + silverMineral1X);
                                        telemetry.update();
                                        goldPos = -1;
                                    }
                                } else if (silverMineral2X != -987654) {
                                    telemetry.addData("Gold Mineral Position", "Right, x pos " + goldMineralX);
                                    telemetry.update();
                                    goldPos = 1;
                                }
                            }
                        }
                        catch(Exception p_exception) {
                            telemetry.addData("Error", "The Depot is in the frame and could not be filtered.  Please adjust the camera accordingly");
                            telemetry.update();
                        }
                    }
                    telemetry.update();
                    while (!(telemetry.update())) {}
                }
                else {
                    if(startTime > elapsedTime.time() + 10) {
                        telemetry.addData("Error: ", "No objects could be found.  Please consider adjusting the camera view on the field, unless this is " +
                                "a competition and it's too late, in which case your season just ended.");
                        telemetry.update();
                        break;
                    }
                }
            }
            else {
                telemetry.addData("Error: ", "The detector could not be initialized.  Please consider upgrading your phones and/or your programmer.");
                telemetry.update();
            }
        }
        return goldPos;
    }

    private void vuforiaInit() { // Initialize Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        // parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Wc1"); // Use external camera
        vuforia = new GOFVuforiaLocalizer(parameters);
    }

    private void detectInit() { // Initialize TensorFlow detector
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        detector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        detector.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        if (!(detector == null)) {
            detector.activate(); // Begin detection
        }
    }
}
