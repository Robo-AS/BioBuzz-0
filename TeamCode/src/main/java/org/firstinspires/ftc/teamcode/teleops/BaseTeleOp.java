package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.commands.AutoAimDriveCommand;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDriveCommand;
import org.firstinspires.ftc.teamcode.subs.BlockerServo;
import org.firstinspires.ftc.teamcode.subs.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subs.FarasSubsystem;
import org.firstinspires.ftc.teamcode.subs.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subs.HoodServo;
import org.firstinspires.ftc.teamcode.subs.Intake;
import org.firstinspires.ftc.teamcode.subs.IntakeServo;
import org.firstinspires.ftc.teamcode.subs.LedSubsystem;
import org.firstinspires.ftc.teamcode.subs.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subs.ShotTable;

public abstract class BaseTeleOp extends RobotOpMode {

    public static double AIM_TRIGGER_THRESHOLD = 0.3;
    public static boolean DEBUG_TELEMETRY = true;

    protected FollowerSubsystem follower;
    protected DriveSubsystem drive;
    protected Intake intake;
    protected FarasSubsystem faras;
    protected IntakeServo intakeServo;

    protected ShooterSubsystem shooterSubsystem;
    protected BlockerServo blockerServo;
    protected LedSubsystem leds;
    protected HoodServo hood;

    public static boolean INTAKE_DOWN_IS_B = true;
    protected GamepadEx driver;
    protected AutoAimDriveCommand autoAim;

    protected abstract Pose startPose();
    protected abstract double goalX();
    protected abstract double goalY();

    protected Command createDefaultDriveCommand() {
        return new FieldCentricDriveCommand(drive, driver);
    }

    protected boolean showMotorDebugTelemetry() {
        return DEBUG_TELEMETRY;
    }

    @Override
    protected void initRobot() {
        follower = new FollowerSubsystem(hardwareMap);
        follower.setPose(startPose());

        drive = new DriveSubsystem(hardwareMap, follower);
        drive.setGoal(goalX(), goalY());

        intake = new Intake(hardwareMap);
        faras = new FarasSubsystem(hardwareMap);
        intakeServo = new IntakeServo(hardwareMap);
        driver = new GamepadEx(gamepad1);

        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        blockerServo = new BlockerServo(hardwareMap);
        hood = new HoodServo(hardwareMap);
        leds = new LedSubsystem(hardwareMap);

        blockerServo.blocked();
        leds.setColor(LedSubsystem.GREEN);

        autoAim = new AutoAimDriveCommand(drive, driver);
        drive.setDefaultCommand(createDefaultDriveCommand());

        new Trigger(() -> gamepad1.options)
                .whenActive(new InstantCommand(() -> drive.resetPose(startPose())));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(autoAim);

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() ->
                        drive.setGoal(drive.mecanum().getGoalX() - 3, drive.mecanum().getGoalY())));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() ->
                        drive.setGoal(drive.mecanum().getGoalX() + 3, drive.mecanum().getGoalY())));

        // R1: Toggle Făraș (se lasă la prima apăsare, se ridică la a doua)
        // Pornește automat și intake-ul pentru fluiditate când e jos
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    faras.toggle();
                    boolean down = faras.isDown();
                    setIntakeServoDown(down);
                    if (down) intake.setPower(Intake.INTAKE_POWER); else intake.stop();
                }, faras, intakeServo, intake));

        configureBindings();
    }

    protected void configureBindings() {
        driver.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(intake.intakeCommand());

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(intakeServo.toggleCommand());

        // R2: AutoAim + Wait 0.5s + Blocker notblocked + IntakeServo to B + Feeder ON + Intake ON
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > AIM_TRIGGER_THRESHOLD)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(autoAim::schedule), // Pornește țintirea automată imediat
                        new WaitCommand(200),                  // Așteaptă 0.5 secunde
                        new InstantCommand(blockerServo::notblocked, blockerServo),
                        new InstantCommand(intakeServo::toB, intakeServo),
                        new InstantCommand(() -> shooterSubsystem.setFeeder(true), shooterSubsystem),
                        new InstantCommand(() -> intake.setPower(1), intake),
                        new InstantCommand(() -> leds.setColor(LedSubsystem.VIOLET), leds)
                ))
                .whenInactive(new SequentialCommandGroup(
                        new InstantCommand(autoAim::cancel),   // Oprește țintirea
                        new InstantCommand(blockerServo::blocked, blockerServo),
                        new InstantCommand(intakeServo::toA, intakeServo),
                        new InstantCommand(() -> shooterSubsystem.setFeeder(false), shooterSubsystem),
                        new InstantCommand(intake::stop, intake)
                ));

        // L2: IntakeServo to B + Intake ON + Feeder ON
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(intakeServo::toB, intakeServo),
                        new InstantCommand(() -> intake.setPower(1), intake),
                        new InstantCommand(() -> shooterSubsystem.setFeeder(true), shooterSubsystem),
                        new InstantCommand(() -> leds.setColor(LedSubsystem.BLUE), leds)
                ))
                .whenInactive(new SequentialCommandGroup(
                        new InstantCommand(intakeServo::toA, intakeServo),
                        new InstantCommand(intake::stop, intake),
                        new InstantCommand(() -> shooterSubsystem.setFeeder(false), shooterSubsystem)
                ));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(shooterSubsystem::toggleFeeder, shooterSubsystem));

        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(shooterSubsystem::toggleflywheel, shooterSubsystem));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(blockerServo::blocked, blockerServo));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(blockerServo::notblocked, blockerServo));
    }

    protected void setIntakeServoDown(boolean down) {
        if (down == INTAKE_DOWN_IS_B) intakeServo.toB(); else intakeServo.toA();
    }

    @Override
    protected void onStart() {
        schedule(intakeServo.toACommand());
        blockerServo.blocked();
        leds.setColor(LedSubsystem.WHITE);
    }

    private void updateLeds() {
        if (autoAim != null && autoAim.isScheduled()) {
            leds.setColor(LedSubsystem.VIOLET);
        } else if (intakeServo != null && intakeServo.isAtB()) {
            leds.setColor(LedSubsystem.BLUE);
        } else {
            leds.setColor(LedSubsystem.WHITE);
        }
    }

    private void updateAutomaticShooting() {
        Pose pose = drive.pose();
        double dist = ShotTable.distance(pose.x(), pose.y(), goalX(), goalY());

        double targetHood = ShotTable.hood(dist);
        hood.setPosition(targetHood);

        double targetRpm = ShotTable.rpm(dist);
        shooterSubsystem.updateFlywheelRpm(targetRpm);
    }

    @Override
    protected void addTelemetry() {
        updateAutomaticShooting();
        updateLeds();

        Pose pose = drive.pose();
        boolean aiming = autoAim.isScheduled();
        double dist = ShotTable.distance(pose.x(), pose.y(), goalX(), goalY());

        telemetry.addData("Auto aim", aiming);
        telemetry.addData("Distance to goal", "%.1f in", dist);
        telemetry.addData("Auto Hood", "%.3f", ShotTable.hood(dist));
        telemetry.addData("Target RPM", "%.0f", ShotTable.rpm(dist));

        if (!aiming) {
            telemetry.addData("Hold error (deg)", Math.toDegrees(drive.headingHold().lastError()));
        }

        drive.mecanum().aimTelemetry(telemetry, pose.x(), pose.y(), pose.heading());
    }
}