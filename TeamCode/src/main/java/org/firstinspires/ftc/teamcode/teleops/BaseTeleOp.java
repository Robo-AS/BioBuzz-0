package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.commands.AutoAimDriveCommand;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDriveCommand;
import org.firstinspires.ftc.teamcode.subs.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subs.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subs.Intake;

/**
 * Shared competition TeleOp. Each alliance TeleOp only supplies a start pose and a goal.
 *
 * gamepad1:
 *   left stick      field-centric translation
 *   right stick X   turn (heading hold when released)
 *   right trigger   hold for auto aim
 *   right bumper    hold to intake
 *   left bumper     hold to outtake
 *   options         re-zero pose to the start spot
 */
public abstract class BaseTeleOp extends RobotOpMode {

    public static double AIM_TRIGGER_THRESHOLD = 0.3;
    public static boolean DEBUG_TELEMETRY = true;

    protected FollowerSubsystem follower;
    protected DriveSubsystem drive;
    protected Intake intake;
    protected GamepadEx driver;
    protected AutoAimDriveCommand autoAim;

    protected abstract Pose startPose();
    protected abstract double goalX();
    protected abstract double goalY();

    /** Override to add bindings for new mechanisms. */
    protected void configureBindings() {}

    @Override
    protected void initRobot() {
        // Order matters: follower first, then drive (so MecanumDrive's motor directions win).
        follower = new FollowerSubsystem(hardwareMap);
        follower.setPose(startPose());

        drive = new DriveSubsystem(hardwareMap, follower);
        drive.setGoal(goalX(), goalY());

        intake = new Intake(hardwareMap);
        driver = new GamepadEx(gamepad1);

        // ---- Drive ----
        autoAim = new AutoAimDriveCommand(drive, driver);
        drive.setDefaultCommand(new FieldCentricDriveCommand(drive, driver));

        new Trigger(() -> gamepad1.right_trigger > AIM_TRIGGER_THRESHOLD)
                .whileActiveOnce(autoAim);

        // No requirements, so pressing it while aiming doesn't cancel the aim.
        new Trigger(() -> gamepad1.options)
                .whenActive(new InstantCommand(() -> drive.resetPose(startPose())));

        // ---- Intake ----
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(intake.intakeCommand());
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(intake.outtakeCommand());

        configureBindings();
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

        if (DEBUG_TELEMETRY) {
            telemetry.addData("Follower mode", follower.follower().mode());
            drive.motorTelemetry(telemetry);
        }
    }
}
