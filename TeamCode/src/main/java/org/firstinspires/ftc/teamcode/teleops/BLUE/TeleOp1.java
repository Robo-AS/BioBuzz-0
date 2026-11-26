package org.firstinspires.ftc.teamcode.teleops.BLUE;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subs.IntakeServo;
import org.firstinspires.ftc.teamcode.subs.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.teleops.BaseTeleOp;

@TeleOp(name = "TeleOp1")
public class TeleOp1 extends BaseTeleOp {
    private static final PoseFactory P = PoseFactory.degrees();
    private static final Pose START = P.of(132, 9, 90);

    private static final double GOAL_X = 80;
    private static final double GOAL_Y = 71;

    private IntakeServo intakeServo;
    private ShooterSubsystem shooterSubsystem; // Declarăm noul subsystem

    @Override protected Pose startPose() { return START; }
    @Override protected double goalX() { return GOAL_X; }
    @Override protected double goalY() { return GOAL_Y; }

    @Override
    protected void configureBindings() {
        // --- INTAKE EXISTENT (acționat pe B) ---
        driver.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(intake.intakeCommand());

        // --- SERVO INTAKE (acționat pe A) ---
        intakeServo = new IntakeServo(hardwareMap);
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(intakeServo.toggleCommand());

        // --- FEEDER ȘI LAUNCHER NOU ---
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        // X: pornește/oprește feeder-ul
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(shooterSubsystem::toggleFeeder, shooterSubsystem));

        // Y: pornește/oprește launcher-ul
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(shooterSubsystem::toggleLauncher, shooterSubsystem));
    }

    @Override
    protected void onStart() {
        schedule(intakeServo.toACommand());
    }

    @Override
    protected void addTelemetry() {
        super.addTelemetry();
        telemetry.addData("Intake servo", intakeServo.isAtB()
                ? "B (" + IntakeServo.POSITION_B + ")"
                : "A (" + IntakeServo.POSITION_A + ")");
    }
}