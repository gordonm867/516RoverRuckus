package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

@TeleOp(name="GOFPotentiometerTest",group="GOFTests")
// @Disabled
public class GOFBoxPotentiometerPoses extends LinearOpMode {
    private GOFHardware robot = GOFHardware.getInstance();
    private volatile OpModeManagerImpl manager = (OpModeManagerImpl) this.internalOpModeServices;
    private boolean doTelemetry = true;
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Thread update = new Thread() {
            @Override
            public synchronized void run() {
                while(!doTelemetry) {
                    try {
                        sleep(100);
                    }
                    catch(Exception p_exception) {
                        Thread.currentThread().interrupt();
                    }
                }
                String active = manager.getActiveOpModeName();
                while(doTelemetry && manager.getActiveOpModeName().equalsIgnoreCase(active)) {
                    try {
                        telemetry.addData("Potentiometer", robot.boxPotentiometer.getVoltage());
                        telemetry.addData("Angle", (robot.boxPotentiometer.getVoltage() / 3.3) * 180);
                    }
                    catch (Exception p_exception) {
                        telemetry.addData("Uh oh", "The driver controller was unable to communicate via telemetry.  For help, please seek a better programmer.");
                    }
                    telemetry.update();
                }
            }
        };
        waitForStart();
        update.start();
        while(opModeIsActive()) {
            robot.box.setPower(Range.clip(gamepad1.left_stick_y * ((gamepad1.dpad_down || gamepad1.dpad_up) ? 0.25 : 1), -1, 1));
        }
    }
}
