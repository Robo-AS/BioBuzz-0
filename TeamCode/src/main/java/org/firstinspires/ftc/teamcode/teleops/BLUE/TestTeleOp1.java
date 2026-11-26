package org.firstinspires.ftc.teamcode.teleops.BLUE;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.RawFieldCentricDriveCommand;

/**
 * Exactly TeleOp1 (same start pose, goal, buttons, intake, servo, shooter, auto aim, telemetry),
 * but the drivetrain never uses the motor encoders and has no motion profiling:
 *  - free roam: raw field-centric power (RawFieldCentricDriveCommand)
 *  - auto aim: unchanged, already raw power (RUN_WITHOUT_ENCODER)
 *  - no motor velocity telemetry (that reads the encoders)
 * Pose/heading still come from the Pinpoint odometry, which field-centric and auto aim need.
 */
@TeleOp(name = "TestTeleOp1")
public class TestTeleOp1 extends TeleOp1 {

    @Override
    protected Command createDefaultDriveCommand() {
        return new RawFieldCentricDriveCommand(drive, driver);
    }

    @Override
    protected boolean showMotorDebugTelemetry() {
        return false;
    }
}
