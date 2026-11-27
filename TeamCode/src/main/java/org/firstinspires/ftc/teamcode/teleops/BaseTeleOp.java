package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.commands.AutoAimDriveCommand;
import org.firstinspires.ftc.teamcode.drive.HeadingHold;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDriveCommand;
import org.firstinspires.ftc.teamcode.subs.BlockerServo;
import org.firstinspires.ftc.teamcode.subs.CameraServo;
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
    /** R2: time the flywheel gets to spin up before the blocker opens (ms). */
    public static long FLYWHEEL_SPINUP_MS = 1000;
    /** Interpolation (ShotTable) OFF for now: fixed shot. Set USE_SHOT_TABLE = true to go back. */
    public static boolean USE_SHOT_TABLE = false;
    public static double FIXED_RPM = 3800;
    public static double FIXED_HOOD = 0.47;
    /** L2 released: run the transfer (feeder) backwards for this long (ms). */
    public static long TRANSFER_REVERSE_MS = 1;
    /** Options resets the pose (auto aim reference) here, for both RED and BLUE. */
    public static final Pose RESET_POSE = PoseFactory.degrees().of(132, 9, 90);
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
    protected CameraServo camera;

    public static boolean INTAKE_DOWN_IS_B = true;
    protected GamepadEx driver;
    protected AutoAimDriveCommand autoAim;

    protected abstract Pose startPose();
    protected abstract double goalX();
    protected abstract double goalY();

    protected Command createDefaultDriveCommand() {
        return new FieldCentricDriveCommand(drive, driver);
    }

    /** Y = odometry reset. The Shooter Tuner turns this off because it uses Y for the flywheel. */
    protected boolean resetOnY() {
        return true;
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
        faras = new FarasSubsystem(hardwareMap); // needed by R1 below (was commented out -> R1 would crash)
        intakeServo = new IntakeServo(hardwareMap);
        driver = new GamepadEx(gamepad1);

        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        blockerServo = new BlockerServo(hardwareMap);
        hood = new HoodServo(hardwareMap);
        camera = new CameraServo(hardwareMap); // goes to CameraServo.INIT_POS (0.8) at INIT
        leds = new LedSubsystem(hardwareMap);

        blockerServo.blocked();
        leds.setColor(LedSubsystem.GREEN);

        autoAim = new AutoAimDriveCommand(drive, driver);
        drive.setDefaultCommand(createDefaultDriveCommand());

        // Y = full odometry reset (OPTIONS in the Shooter Tuner) (like restarting the robot). Robot must be ON (132, 9, 90)
        // and STILL: stops the drive + auto aim, recalibrates the Pinpoint IMU, waits until it's
        // READY (min 0.3 s, max 1 s), then sets the pose to RESET_POSE.
        final long[] resetStartMs = {0};
        Command resetOdometry = new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            autoAim.cancel();
                            drive.stop();
                            drive.recalibrateOdometry();
                            resetStartMs[0] = System.currentTimeMillis();
                            leds.setColor(LedSubsystem.RED);
                        }, drive),
                        new WaitUntilCommand(() -> {
                            long t = System.currentTimeMillis() - resetStartMs[0];
                            return t >= 300 && (drive.odometryReady() || t >= 1000);
                        }),
                        new InstantCommand(() -> {
                            drive.resetPose(RESET_POSE);
                            leds.setColor(LedSubsystem.WHITE);
                        }, drive)
                );
        // Heading hold starts OFF every TeleOp (static, so it must be reset here).
        HeadingHold.ENABLED = false;
        if (resetOnY()) {
            driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(resetOdometry);
            // OPTIONS = heading hold ON <-> OFF (when turned on it holds the current heading)
            new Trigger(() -> gamepad1.options).whenActive(new InstantCommand(() -> {
                HeadingHold.ENABLED = !HeadingHold.ENABLED;
                drive.headingHold().reset();
            }));
        } else {
            new Trigger(() -> gamepad1.options).whenActive(resetOdometry);
        }

        // X = auto aim toggle -- DISABLED for now
        // driver.getGamepadButton(GamepadKeys.Button.X)
        //         .toggleWhenPressed(autoAim);

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

        // R2 (hold): [AutoAim DISABLED for now] flywheel ON -> wait FLYWHEEL_SPINUP_MS so it reaches speed ->
        //            Intake ON + IntakeServo to B + Blocker opens -> Wait 400 -> Feeder ON
        // release:   everything off, blocker closed, flywheel OFF,
        //            Feeder BACKWARDS for TRANSFER_REVERSE_MS, then OFF
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > AIM_TRIGGER_THRESHOLD)
                .whenActive(new SequentialCommandGroup(
                        // new InstantCommand(autoAim::schedule),   // auto aim DISABLED for now
                        new InstantCommand(() -> shooterSubsystem.setFlywheel(true), shooterSubsystem),
                        new WaitCommand(FLYWHEEL_SPINUP_MS),
                        new InstantCommand(() -> intake.setPower(1), intake),
                        new InstantCommand(intakeServo::toB, intakeServo),
                        new InstantCommand(blockerServo::notblocked, blockerServo),
                        new WaitCommand(400),
                        new InstantCommand(() -> shooterSubsystem.setFeeder(true), shooterSubsystem),

                        new InstantCommand(() -> leds.setColor(LedSubsystem.VIOLET), leds)
                ))
                .whenInactive(new SequentialCommandGroup(
                        // new InstantCommand(autoAim::cancel),   // auto aim DISABLED for now
                        new InstantCommand(blockerServo::blocked, blockerServo),
                        new InstantCommand(intakeServo::toA, intakeServo),
                        new InstantCommand(() -> shooterSubsystem.setFlywheel(false), shooterSubsystem),
                        new InstantCommand(intake::stop, intake),
                        new InstantCommand(() -> shooterSubsystem.setFeederPower(-1.0), shooterSubsystem),
                        new WaitCommand(TRANSFER_REVERSE_MS),
                        new InstantCommand(() -> shooterSubsystem.setFeeder(false), shooterSubsystem)
                ));

        // L2 (hold): IntakeServo to B + Intake ON + Feeder ON (blocker stays closed, no flywheel)
        // release:   IntakeServo to A + Intake OFF + Feeder BACKWARDS for TRANSFER_REVERSE_MS, then OFF
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
                        new InstantCommand(() -> shooterSubsystem.setFeederPower(-1.0), shooterSubsystem),
                        new WaitCommand(TRANSFER_REVERSE_MS),
                        new InstantCommand(() -> shooterSubsystem.setFeeder(false), shooterSubsystem)
                ));

        // L1 (hold): EJECT -> intake + feeder full speed BACKWARDS to spit out unwanted balls.
        // release:   both stop.
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    intake.setPower(-1.0);
                    shooterSubsystem.setFeederPower(-1.0);
                }, intake, shooterSubsystem))
                .whenReleased(new InstantCommand(() -> {
                    intake.stop();
                    shooterSubsystem.setFeeder(false);
                }, intake, shooterSubsystem));

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
        shooterSubsystem.setFlywheel(false); // flywheel only runs while R2 is held
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

    /** Distance-based flywheel RPM + hood every loop. Tuning TeleOps return false to control them by hand. */
    protected boolean useAutomaticShooting() {
        return true;
    }

    private void updateAutomaticShooting() {
        Pose pose = drive.pose();
        double targetHood = FIXED_HOOD;
        double targetRpm = FIXED_RPM;
        if (USE_SHOT_TABLE) {
            // Picks the table for the side of the hive the robot is on, then interpolates by distance.
            targetHood = ShotTable.hood(pose.x(), pose.y(), goalX(), goalY());
            targetRpm = ShotTable.rpm(pose.x(), pose.y(), goalX(), goalY());
        }
        hood.setPosition(targetHood);
        shooterSubsystem.updateFlywheelRpm(targetRpm);
    }

    @Override
    protected void addTelemetry() {
        if (useAutomaticShooting()) updateAutomaticShooting();
        updateLeds();

        Pose pose = drive.pose();
        boolean aiming = autoAim.isScheduled();
        double dist = ShotTable.distance(pose.x(), pose.y(), goalX(), goalY());

        telemetry.addData("Auto aim", aiming);
        telemetry.addData("Odometry", drive.odometryStatus());
        telemetry.addData("Motor direction fixes", drive.mecanum().directionFixes);
        telemetry.addData("Heading hold", HeadingHold.ENABLED ? "ON (OPTIONS = off)" : "OFF (OPTIONS = on)");
        telemetry.addData("Distance to goal", "%.1f in  (%s side table)", dist, ShotTable.sideName(pose.y(), goalY()));
        telemetry.addData("Hood", "%.3f%s", USE_SHOT_TABLE ? ShotTable.hood(pose.x(), pose.y(), goalX(), goalY()) : FIXED_HOOD,
                USE_SHOT_TABLE ? " (table)" : " (fixed)");
        telemetry.addData("Target RPM", "%.0f%s", USE_SHOT_TABLE ? ShotTable.rpm(pose.x(), pose.y(), goalX(), goalY()) : FIXED_RPM,
                USE_SHOT_TABLE ? " (table)" : " (fixed)");
        telemetry.addData("Actual RPM", "%.0f", shooterSubsystem.getActualRpm());
        telemetry.addData("Camera servo", camera.isConfigured()
                ? String.format("%.3f", camera.getPosition())
                : "NOT in config (add a Servo named \"camera\")");

        if (!aiming) {
            telemetry.addData("Hold error (deg)", Math.toDegrees(drive.headingHold().lastError()));
        }

        drive.mecanum().aimTelemetry(telemetry, pose.x(), pose.y(), pose.heading());

        faras.telemetry(telemetry);
    }
}