package org.firstinspires.ftc.teamcode.autos.BLUE;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class BlueTest extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(87.9039, 131.6771, 90);
    private final Pose path1 = poseFactory.of(87.7063, 113.9454, 90);
    private final Pose point2Start = poseFactory.of(87.7063, 113.9454, 90);
    private final Pose point2 = poseFactory.of(88.2352, 134.2903, 0);
    private final Pose point3Start = poseFactory.of(88.2352, 134.2903, 0);
    private final Pose point3 = poseFactory.of(132.4953, 134.8418, 0);
    private final Pose point4Start = poseFactory.of(132.4953, 134.8418, 0);
    private final Pose point4 = poseFactory.of(83.0475, 20.3248, 270);
    private final Pose point4Control1 = poseFactory.of(111.1099, 82.9889, 0);
    private final Pose point4Control2 = poseFactory.of(126.079, 41.408, 0);
    private final Pose point5 = poseFactory.of(94.6762, 17.7553, 270);
    private final Pose point6 = poseFactory.of(83.5216, 20.4009, 270);
    private final Pose point7Start = poseFactory.of(83.5216, 20.4009, 270);
    private final Pose point7 = poseFactory.of(122.1442, 94.8738, 0);
    private final Pose point7Control1 = poseFactory.of(110.491, 25.9946, 0);
    private final Pose point8Start = poseFactory.of(122.1442, 94.8738, 0);
    private final Pose point8 = poseFactory.of(87.3239, 114.549, 90);
    private final Pose point9 = poseFactory.of(129.4892, 48.1952, 90);
    private final Pose point9Control1 = poseFactory.of(111.1148, 106.8592, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8()),
                follow(follower, path9())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4Control2, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point4, point5).constant(point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).constant(point6);
    }

    public Path path7() {
        return Paths.curve(point7Start, point7Control1, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.curve(point8, point9Control1, point9).constant(point9);
    }
}
