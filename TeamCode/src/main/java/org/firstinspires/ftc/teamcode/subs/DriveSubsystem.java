package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HeadingHold;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

/**
 * TeleOp drivetrain subsystem. Wraps MecanumDrive (drive math + auto aim untouched)
 * and reads the pose from FollowerSubsystem.
 *
 * IMPORTANT: build this AFTER FollowerSubsystem, so MecanumDrive's motor directions win.
 * Do NOT build it in Auto — there Pedro's own motor directions must be used.
 */
public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive mecanum;
    private final FollowerSubsystem localizer;
    private final HeadingHold headingHold = new HeadingHold();

    public DriveSubsystem(HardwareMap hardwareMap, FollowerSubsystem localizer) {
        this.localizer = localizer;
        this.mecanum = new MecanumDrive(hardwareMap);
    }

    public MecanumDrive mecanum() { return mecanum; }
    public HeadingHold headingHold() { return headingHold; }
    public Pose pose() { return localizer.pose(); }

    public void setGoal(double x, double y) {
        mecanum.setGoal(x, y);
    }

    /** Options button: put the robot on its start spot, then call this. */
    public void resetPose(Pose startPose) {
        localizer.setPose(startPose);
        headingHold.reset();
        mecanum.resetFreeRoamProfile();
    }

    public void motorTelemetry(Telemetry telemetry) {
        telemetry.addData("FL mode", mecanum.frontLeft.getMode());
        telemetry.addData("FR mode", mecanum.frontRight.getMode());
        telemetry.addData("RL mode", mecanum.rearLeft.getMode());
        telemetry.addData("RR mode", mecanum.rearRight.getMode());
        telemetry.addData("FL speed (ticks/s)", "%.0f", Math.abs(mecanum.frontLeft.getVelocity()));
        telemetry.addData("FR speed (ticks/s)", "%.0f", Math.abs(mecanum.frontRight.getVelocity()));
        telemetry.addData("RL speed (ticks/s)", "%.0f", Math.abs(mecanum.rearLeft.getVelocity()));
        telemetry.addData("RR speed (ticks/s)", "%.0f", Math.abs(mecanum.rearRight.getVelocity()));
    }
}
