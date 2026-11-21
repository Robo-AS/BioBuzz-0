package org.firstinspires.ftc.teamcode.servotest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/*
 * TEST TOOL — reads the "Sweep" continuous rotation servo's position from its analog
 * feedback wire (Axon-style). The servo gets NO power from this OpMode: turn it by hand
 * and note the start and end positions.
 *
 * Robot Configuration: the feedback wire's analog port must be set up as an
 * "Analog Input" device named "Sweep".
 */
@TeleOp(name = "Sweep read", group = "Test")
public class SweepEncoderReader extends LinearOpMode {

    private static final String ENCODER_NAME = "Sweep";

    @Override
    public void runOpMode() {
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, ENCODER_NAME);

        while (opModeInInit() || opModeIsActive()) {
            double voltage = encoder.getVoltage();
            double degrees = voltage / encoder.getMaxVoltage() * 360.0;
            telemetry.addData("Sweep position", "%.1f°  (%.3f V)", degrees, voltage);
            telemetry.update();
        }
    }
}
