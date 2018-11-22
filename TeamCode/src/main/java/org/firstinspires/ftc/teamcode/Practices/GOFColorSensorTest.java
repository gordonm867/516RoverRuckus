package org.firstinspires.ftc.teamcode.Practices;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name="ColorSensorTest",group="GOFTests")
@Disabled

public class GOFColorSensorTest extends LinearOpMode {

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    public void runOpMode() {
        sensorColor = hardwareMap.get(ColorSensor.class, "cd0");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "cd0");

        waitForStart();

        double ratio;
        while(opModeIsActive()) {
            ratio = sensorColor.red() / sensorColor.blue(); // GOLD: RED TO BLUE = 2:1, SILVER: RED TO BLUE = 1:1, PURPLE: RED TO BLUE 1:2
            if(Math.abs((2 - ratio)) > Math.abs((1 - ratio))) {
                if(Math.abs((0.5 - ratio)) > Math.abs(1 - ratio)) {
                    telemetry.addData("Cube Type: ", "Silver");
                }
                else {
                    telemetry.addData("Cube Type: ", "Purple");
                }
            }
            else {
                telemetry.addData("Cube Type: ", "Gold");
            }
            telemetry.update();
        }
    }


}
