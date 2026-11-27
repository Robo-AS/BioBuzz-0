package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HeadingHold;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive mecanum;
    private final FollowerSubsystem localizer;
    private final HeadingHold headingHold = new HeadingHold();
    private final GoBildaPinpointDriver pinpoint; // same device Pedro uses ("pinpoint")

    public DriveSubsystem(HardwareMap hardwareMap, FollowerSubsystem localizer) {
        this.localizer = localizer;
        this.mecanum = new MecanumDrive(hardwareMap);
        this.pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, "pinpoint");
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
        mecanum.resetAim();
    }

    /** All drive motors to 0. */
    public void stop() {
        mecanum.frontLeft.setPower(0);
        mecanum.frontRight.setPower(0);
        mecanum.rearLeft.setPower(0);
        mecanum.rearRight.setPower(0);
    }

    /**
     * Same as a power cycle for the odometry: zeroes the Pinpoint AND recalibrates its IMU.
     * The robot MUST be completely still for ~0.25 s afterwards. Call resetPose() once it's READY.
     */
    public void recalibrateOdometry() {
        if (pinpoint != null) pinpoint.resetPosAndIMU();
    }

    /** True when the Pinpoint finished calibrating (or if it can't be read directly). */
    public boolean odometryReady() {
        return pinpoint == null || pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public String odometryStatus() {
        return pinpoint == null ? "?" : pinpoint.getDeviceStatus().toString();
    }

    public void motorTelemetry(Telemetry telemetry) {
        telemetry.addData("FL power", "%.2f", mecanum.frontLeft.getPower());
        telemetry.addData("FR power", "%.2f", mecanum.frontRight.getPower());
        telemetry.addData("RL power", "%.2f", mecanum.rearLeft.getPower());
        telemetry.addData("RR power", "%.2f", mecanum.rearRight.getPower());
    }
}