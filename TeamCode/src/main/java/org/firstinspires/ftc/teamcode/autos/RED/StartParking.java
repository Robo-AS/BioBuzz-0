package org.firstinspires.ftc.teamcode.autos.RED;

import static com.pedropathing.api.Paths.line;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subs.FollowerSubsystem;

@Autonomous
public class StartParking extends RobotOpMode {
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(38.7, 8.6, 90);
    private final Pose park = p.of(9.57, 110.99, 90);
    private final Pose shooting1 = p.of(56.05, 25.52, 90);
    private final Pose pick1 = p.of(7.6, 7.6, 180);

    // Auto uses ONLY the follower (no DriveSubsystem) so Pedro's motor directions are used.
    private FollowerSubsystem follower;

    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }

    private Path shoot() {
        return line(startPose, shooting1).linear(startPose, shooting1);
    }

    private Path pickup() {
        return line(startPose, pick1).tangent();
    }

    private Path returnpickstart() {
        return line(pick1, startPose).linear(pick1, startPose);
    }

    private Path returnstart() {
        return line(shooting1, startPose).linear(shooting1, startPose);
    }

    private Command follow(Path path) {
        return new FollowPathCommand(follower, path);
    }

    private Command autoRoutine() {
        return new SequentialCommandGroup(
                follow(shoot()),
                follow(returnstart()),
                follow(pickup()),
                follow(returnpickstart()),
                follow(shoot()),
                follow(returnstart()),
                follow(park())
        );
    }

    @Override
    protected void initRobot() {
        follower = new FollowerSubsystem(hardwareMap);
        follower.setPose(startPose);
        follower.periodic(); // same as the old follower.update() in init
    }

    @Override
    protected void onStart() {
        schedule(autoRoutine());
    }

    @Override
    protected void addTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading", pose.heading());
    }
}
