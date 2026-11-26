package org.firstinspires.ftc.teamcode.teleops.test;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.subs.FarasSubsystem;

/*
 * TEST TOOL — "faras" continuous servo, TIME-BASED (no encoder).
 *
 *   BEFORE INIT  put the faras UP. After START it holds UP.
 *   A            UP -> DOWN (runs DOWN_TIME_MS, then holds), press again -> UP
 *   dpad up      +50 ms on the time of the current direction
 *   dpad down    -50 ms on the time of the current direction
 *
 * Telemetry shows the DOWN / UP times. Once right, copy them into FarasSubsystem.
 * Times and powers are also live in FTC Dashboard -> Config -> "Faras".
 */
@TeleOp(name = "Faras Test", group = "Test")
public class FarasTeleOp extends RobotOpMode {

    private FarasSubsystem faras;

    @Override
    protected void initRobot() {
        faras = new FarasSubsystem(hardwareMap);
        GamepadEx driver = new GamepadEx(gamepad1);

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(faras.toggleCommand());
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> faras.trimTime(50)));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> faras.trimTime(-50)));
    }

    @Override
    protected void addTelemetry() {
        faras.telemetry(telemetry);
    }
}
