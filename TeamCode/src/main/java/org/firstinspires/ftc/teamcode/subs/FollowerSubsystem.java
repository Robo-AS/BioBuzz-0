package org.firstinspires.ftc.teamcode.subs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Owns the Pedro Follower: localization (Pinpoint) in every OpMode, path following in Auto.
 * The pose is read ONCE per loop in periodic() and cached, so every command sees the same pose.
 */
public class FollowerSubsystem extends SubsystemBase {

    private final Follower follower;
    private Pose pose;

    public FollowerSubsystem(HardwareMap hardwareMap) {
        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        pose = follower.pose();
    }

    @Override
    public void periodic() {
        follower.update();
        pose = follower.pose();
    }

    public void setPose(Pose newPose) {
        follower.setPose(newPose);
        pose = follower.pose(); // commands running this loop already see the new pose
    }

    public Pose pose() {
        return pose;
    }

    public Follower follower() {
        return follower;
    }
}
