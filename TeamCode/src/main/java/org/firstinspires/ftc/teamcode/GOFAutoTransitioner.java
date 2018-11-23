package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TAG;

public class GOFAutoTransitioner extends Thread {

    private static final GOFAutoTransitioner INSTANCE = new GOFAutoTransitioner();

    private OpMode opMode;
    private OpModeManagerImpl manager;
    private String nextOpMode;

    private GOFAutoTransitioner() {
        this.start(); // Start monitoring active opMode
    }

    public synchronized void run() {
        try {
            if (manager != null) {
                while (manager.getActiveOpMode() == opMode) {
                    Thread.sleep(100); // While the initial OpMode is active, sleep the thread to minimize number of checks that occur
                }
                Thread.sleep(1000); // Sleep the thread for one second before initializing the next OpMode to avoid interfering with the FTC apps
                manager.initActiveOpMode(nextOpMode); // Request initialization of the next OpMode
                while(!manager.getActiveOpModeName().equals(nextOpMode)) {} // Wait until next OpMode initialized
                Thread.currentThread().interrupt(); // Stop this thread after next OpMode initialized
            }
            else {
                Thread.currentThread().interrupt(); // If the OpModeManager goes null, stop the thread, since this won't work anyway
            }
        }
        catch(Exception p_exception) {
            if(manager == null) {
                Log.e(TAG, "Automatic transitioner failed with null OpMode manager"); // Add data to logcat for debugging
            }
            else {
                if(manager.getActiveOpModeName().equals(nextOpMode)) {
                    Log.e(TAG, "Automatic transitioner successfully initialized " + nextOpMode); // Note success for debugging
                }
                else {
                    Log.e(TAG, "Automatic transitioner failed with exception", p_exception); // Add data to logcat for debugging
                }
            }
        }
    }

    public static void transitionOnStop(OpMode currentOpMode, String nextOpMode) {
        INSTANCE.newStop(currentOpMode, nextOpMode); // Get new instance of class, and pass parameters to a non-static method
    }

    private synchronized void newStop(OpMode currentOpMode, String nextOpMode) {
        this.opMode = currentOpMode; // Store OpMode of initial class
        this.nextOpMode = nextOpMode; // Store the name of the next intended OpMode
        this.manager = (OpModeManagerImpl) currentOpMode.internalOpModeServices; // Store the OpMode manager
    }
}