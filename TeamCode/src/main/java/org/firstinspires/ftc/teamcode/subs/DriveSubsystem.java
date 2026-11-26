package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HeadingHold;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

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
    }

    public void motorTelemetry(Telemetry telemetry) {
        telemetry.addData("FL power", "%.2f", mecanum.frontLeft.getPower());
        telemetry.addData("FR power", "%.2f", mecanum.frontRight.getPower());
        telemetry.addData("RL power", "%.2f", mecanum.rearLeft.getPower());
        telemetry.addData("RR power", "%.2f", mecanum.rearRight.getPower());
    }
}