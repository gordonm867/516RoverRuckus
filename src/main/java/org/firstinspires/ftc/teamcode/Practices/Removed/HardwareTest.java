/*
package org.firstinspires.ftc.teamcode.Practices.Removed;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class HardwareTest {

    public DcMotor inWheel;
    HardwareMap hwMap;

    // Constructor
    public HardwareTest() {}

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        try {
            inWheel = hwMap.get(DcMotor.class, "inw");
            inWheel.setDirection(DcMotor.Direction.FORWARD);
            inWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            inWheel.setPower(0);
            inWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception p_exception) {
            inWheel = null;
        }
    }

    public void setInPower(double inPower) { // Set intake power
        if (inWheel != null) {
            inPower = Range.clip(inPower, -0.25, 0.25);
            inWheel.setPower(inPower);
        }
    }
} */

