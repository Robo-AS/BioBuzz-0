package org.firstinspires.ftc.teamcode.autos.test;

import static com.pedropathing.api.Paths.line;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subs.FollowerSubsystem;

@Autonomous
public class TestAuto extends RobotOpMode {
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(56, 8, 90);
    private final Pose park = p.of(22.294382022471915, 35.36404494382024, 0);

    // Auto uses ONLY the follower (no DriveSubsystem) so Pedro's motor directions are used.
    private FollowerSubsystem follower;

    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }

    @Override
    protected void initRobot() {
        follower = new FollowerSubsystem(hardwareMap);
        follower.setPose(startPose);
        follower.periodic(); // same as the old follower.update() in init
    }

    @Override
    protected void onStart() {
        schedule(new SequentialCommandGroup(
                new FollowPathCommand(follower, park())
        ));
    }

    @Override
    protected void addTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading", pose.heading());
    }
}
