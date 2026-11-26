package org.firstinspires.ftc.teamcode.teleops.BLUE;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subs.BlockerServo;
import org.firstinspires.ftc.teamcode.subs.HoodServo;
import org.firstinspires.ftc.teamcode.subs.IntakeServo;
import org.firstinspires.ftc.teamcode.subs.LedSubsystem;
import org.firstinspires.ftc.teamcode.subs.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.teleops.BaseTeleOp;

@TeleOp(name = "TeleOp1")
public class TeleOp1 extends BaseTeleOp {
    private static final PoseFactory P = PoseFactory.degrees();
    private static final Pose START = P.of(132, 9, 90);

    private static final double GOAL_X = 77;
    private static final double GOAL_Y = 71;


    private ShooterSubsystem shooterSubsystem;
    private BlockerServo blockerServo;
    private LedSubsystem leds;
    private HoodServo hood;

    @Override protected Pose startPose() { return START; }
    @Override protected double goalX() { return GOAL_X; }
    @Override protected double goalY() { return GOAL_Y; }

    @Override
    protected void configureBindings() {
        // --- LED: VERDE CÂND E PE INIT ---
        leds = new LedSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        blockerServo = new BlockerServo(hardwareMap);
        blockerServo.blocked();
        hood= new HoodServo(hardwareMap);
        leds.setColor(LedSubsystem.GREEN);
        // INTAKE SI SERVO
        driver.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(intake.intakeCommand());

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(intakeServo.toggleCommand());
        //R2
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > BaseTeleOp.AIM_TRIGGER_THRESHOLD)
                .whenActive(new ParallelCommandGroup(
                        // Pornește AutoAim în paralel
                        //autoAim,
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                // Blochează, trage și ia
                                new InstantCommand(blockerServo::notblocked, blockerServo),
                                new InstantCommand(intakeServo::toB, intakeServo),
                                new InstantCommand(() -> shooterSubsystem.setFeeder(true), shooterSubsystem),
                                new InstantCommand(() -> intake.setPower(1), intake),
                                new InstantCommand(() -> leds.setColor(LedSubsystem.VIOLET), leds)
                        )
                ))
                .whenInactive(new SequentialCommandGroup(
                        // Oprește AutoAim (îl scoate din program)
                        //new InstantCommand(autoAim::cancel),
                        // Revine la normal
                        new InstantCommand(blockerServo::blocked, blockerServo),
                        new InstantCommand(intakeServo::toA, intakeServo),
                        new InstantCommand(() -> shooterSubsystem.setFeeder(false), shooterSubsystem),
                        new InstantCommand(intake::stop, intake)
                ));
        //L2
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3)
                .whenActive(new SequentialCommandGroup(
                        // Presupunem că vrei ca servo-ul să coboare/schimbe poziția
                        new InstantCommand(intakeServo::toB, intakeServo),
                        new InstantCommand(() -> intake.setPower(1), intake),
                        new InstantCommand(() -> shooterSubsystem.setFeeder(true), shooterSubsystem),
                        new InstantCommand(() -> leds.setColor(LedSubsystem.BLUE), leds)
                ))
                .whenInactive(new SequentialCommandGroup(
                        new InstantCommand(intakeServo::toA, intakeServo),
                        new InstantCommand(intake::stop, intake),
                        new InstantCommand(blockerServo::blocked, blockerServo),
                        new InstantCommand(() -> shooterSubsystem.setFeeder(false), shooterSubsystem)
                ));
        // R1 = faras, bound in BaseTeleOp.
        // --- FEEDER ȘI FLYWHEEL ---


        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(shooterSubsystem::toggleFeeder, shooterSubsystem));

        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(shooterSubsystem::toggleflywheel, shooterSubsystem));

        // --- BLOCKER ---


        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(blockerServo::blocked, blockerServo));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(blockerServo::notblocked, blockerServo));

        // Goal X trim (D-pad left/right) and auto aim (X) are in BaseTeleOp now (default for all TeleOps).
        if (autoAim.isScheduled()) {
            leds.setColor(LedSubsystem.VIOLET);
        }
        // 2. Intake ridicat -> ALBASTRU (dacă poziția ridicat este A, schimbă în !intakeServo.isAtB())
        else if (intakeServo.isAtB()) {
            leds.setColor(LedSubsystem.BLUE);
        }
        // 3. Stare normală -> ALB
        else {
            leds.setColor(LedSubsystem.WHITE);
        }
    }

    @Override
    protected void onStart() {
        schedule(intakeServo.toACommand());
        blockerServo.blocked();
        leds.setColor(LedSubsystem.WHITE);
    }

    @Override
    protected void addTelemetry() {
        super.addTelemetry();
        telemetry.addData("Intake servo", intakeServo.isAtB()
                ? "B (" + IntakeServo.POSITION_B + ")"
                : "A (" + IntakeServo.POSITION_A + ")");
    }
}