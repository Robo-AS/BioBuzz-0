package org.firstinspires.ftc.teamcode.teleops.BLUE;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.RawFieldCentricDriveCommand;
import org.firstinspires.ftc.teamcode.subs.FarasSubsystem;

/**
 * Exactly TeleOp1 (same start pose, goal, buttons, intake, servo, shooter, auto aim, telemetry),
 * but the drivetrain never uses the motor encoders and has no motion profiling:
 *  - free roam: raw field-centric power (RawFieldCentricDriveCommand)
 *  - auto aim: unchanged, already raw power (RUN_WITHOUT_ENCODER)
 *  - no motor velocity telemetry (that reads the encoders)
 * Pose/heading still come from the Pinpoint odometry, which field-centric and auto aim need.
 *
 * Extra: dpad up = "faras" servo 56° <-> 380° (where it is at START = 56°), PD-controlled.
 */
@TeleOp(name = "TestTeleOp1")
public class TestTeleOp1 extends TeleOp1 {

    private FarasSubsystem faras;

    @Override
    protected void configureBindings() {
        super.configureBindings(); // everything TeleOp1 has (B, A, X, Y)

        // DPAD UP: faras 56° <-> 380°
        faras = new FarasSubsystem(hardwareMap);
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(faras.toggleCommand());
    }

    @Override
    protected void addTelemetry() {
        super.addTelemetry();
        faras.telemetry(telemetry);
    }

    @Override
    protected Command createDefaultDriveCommand() {
        return new RawFieldCentricDriveCommand(drive, driver);
    }

    @Override
    protected boolean showMotorDebugTelemetry() {
        return false;
    }
}
