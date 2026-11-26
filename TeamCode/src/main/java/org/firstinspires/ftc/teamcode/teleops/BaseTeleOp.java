package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.commands.AutoAimDriveCommand;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDriveCommand;
import org.firstinspires.ftc.teamcode.subs.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subs.FarasSubsystem;
import org.firstinspires.ftc.teamcode.subs.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subs.Intake;
import org.firstinspires.ftc.teamcode.subs.IntakeServo;

/**
 * Shared competition TeleOp. Each alliance TeleOp only supplies a start pose and a goal.
 *
 * gamepad1 (defaults for ALL TeleOps):
 *   left stick         field-centric translation
 *   right stick X      turn (heading hold when released)
 *   X                  auto aim on / off (back to the goal)
 *   dpad left / right  goal X -3 / +3 (auto aim trim)
 *   right bumper (R1)  faras UP <-> DOWN
 *   options            re-zero pose to the start spot
 * Each TeleOp adds its own mechanism buttons in configureBindings().
 */
public abstract class BaseTeleOp extends RobotOpMode {

    public static double AIM_TRIGGER_THRESHOLD = 0.3;
    public static boolean DEBUG_TELEMETRY = true;

    protected FollowerSubsystem follower;
    protected DriveSubsystem drive;
    protected Intake intake;
    protected FarasSubsystem faras;
    protected IntakeServo intakeServo;

    /** Which IntakeServo position is DOWN (B = 0.9). Set false if DOWN is actually A (0.21). */
    public static boolean INTAKE_DOWN_IS_B = true;
    protected GamepadEx driver;
    protected AutoAimDriveCommand autoAim;

    protected abstract Pose startPose();
    protected abstract double goalX();
    protected abstract double goalY();

    /** Override to add bindings for new mechanisms. */
    protected void configureBindings() {}

    /** Free-roam drive command (default). Override to use a different drive style. */
    protected Command createDefaultDriveCommand() {
        return new FieldCentricDriveCommand(drive, driver);
    }

    /** Show the motor mode/velocity debug lines (velocity comes from the motor encoders). */
    protected boolean showMotorDebugTelemetry() {
        return DEBUG_TELEMETRY;
    }

    @Override
    protected void initRobot() {
        // Order matters: follower first, then drive (so MecanumDrive's motor directions win).
        follower = new FollowerSubsystem(hardwareMap);
        follower.setPose(startPose());

        drive = new DriveSubsystem(hardwareMap, follower);
        drive.setGoal(goalX(), goalY());

        intake = new Intake(hardwareMap);
        faras = new FarasSubsystem(hardwareMap); // must be UP at START; holds UP after START
        intakeServo = new IntakeServo(hardwareMap);
        driver = new GamepadEx(gamepad1);

        // ---- Drive ----
        autoAim = new AutoAimDriveCommand(drive, driver);
        drive.setDefaultCommand(createDefaultDriveCommand());

        /*new Trigger(() -> gamepad1.right_trigger > AIM_TRIGGER_THRESHOLD)
                .whileActiveOnce(autoAim);*/

        // No requirements, so pressing it while aiming doesn't cancel the aim.
        new Trigger(() -> gamepad1.options)
                .whenActive(new InstantCommand(() -> drive.resetPose(startPose())));

        // ---- Intake ----
       /* driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(intake.intakeCommand());
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(intake.outtakeCommand());*/

        // ---- Auto aim: X = on / off (robot turns its back to the goal, driver keeps translation) ----
        driver.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(autoAim);

        // ---- Goal X trim: D-pad left = goal X - 3, D-pad right = goal X + 3 ----
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() ->
                        drive.setGoal(drive.mecanum().getGoalX() - 3, drive.mecanum().getGoalY())));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() ->
                        drive.setGoal(drive.mecanum().getGoalX() + 3, drive.mecanum().getGoalY())));

        // ---- R1: faras + intake servo DOWN and intake motor ON together;
        //      R1 again: faras + intake servo UP and intake motor OFF together ----
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    faras.toggle();
                    boolean down = faras.isDown();
                    setIntakeServoDown(down);
                    if (down) intake.setPower(Intake.INTAKE_POWER); else intake.stop();
                }, faras, intakeServo, intake));

        configureBindings();
    }

    /** Intake servo DOWN or UP (uses INTAKE_DOWN_IS_B to know which position is down). */
    protected void setIntakeServoDown(boolean down) {
        if (down == INTAKE_DOWN_IS_B) intakeServo.toB(); else intakeServo.toA();
    }

    /** At START: intake servo UP (like the faras, which starts UP). TeleOps may override. */
    @Override
    protected void onStart() {
        setIntakeServoDown(false);
    }

    @Override
    protected void addTelemetry() {
        Pose pose = drive.pose();
        boolean aiming = autoAim.isScheduled();

        telemetry.addData("Auto aim", aiming);
        telemetry.addData("Pose", "x=%.1f y=%.1f", pose.x(), pose.y());
        if (!aiming) {
            telemetry.addData("Hold heading (deg)", Math.toDegrees(drive.headingHold().heldHeading()));
            telemetry.addData("Hold error (deg)", Math.toDegrees(drive.headingHold().lastError()));
        }
        drive.mecanum().aimTelemetry(telemetry, pose.x(), pose.y(), pose.heading());
        faras.telemetry(telemetry);

        if (DEBUG_TELEMETRY) telemetry.addData("Follower mode", follower.follower().mode());
        if (showMotorDebugTelemetry()) drive.motorTelemetry(telemetry);
    }
}
