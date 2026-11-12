package org.firstinspires.ftc.teamcode.autos.RED;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subs.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subs.Intake;

/*
 * Order: shooting1 -> return1 -> pick1 (intake on, slower) -> shooting2 -> park
 *
 * Built as flat command sequences on purpose: FTCLib 2.1.1 breaks when a command group
 * is nested inside a parallel group (freezes or crashes), see StartParking.
 */
@Autonomous(name = "RedTest", group = "RED")
public class RedTest extends RobotOpMode {

    /** Intake power while driving pick1. */
    public static double PICK1_INTAKE_POWER = 0.75;
    /** Robot speed on ALL paths (Pedro maxPathSpeed, 1.0 = full speed). */
    public static double DRIVE_SPEED = 0.5;
    /** Extra time the intake keeps running at the end of pick1 (0 = stop as soon as pick1 ends). */
    public static long PICK1_DWELL_MS = 0;

    // Auto uses ONLY the follower (no DriveSubsystem) so Pedro's motor directions are used.
    private FollowerSubsystem follower;
    private Intake intake;
    private String step = "waiting for START";

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(52.8309, 8.6061, 90);
    private final Pose path1 = poseFactory.of(53.0612, 27.8706, 90);
    private final Pose point2Start = poseFactory.of(53.0612, 27.8706, 90);
    private final Pose point2 = poseFactory.of(52.9396, 7.5769, 180);
    private final Pose point3Start = poseFactory.of(52.9396, 7.5769, 180);
    private final Pose point3 = poseFactory.of(9.428, 7.3574, 180);
    private final Pose point4Start = poseFactory.of(9.428, 7.3574, 180);
    private final Pose point4 = poseFactory.of(52.9552, 27.4025, 90);
    private final Pose point5Start = poseFactory.of(52.9552, 27.4025, 90);
    private final Pose point5 = poseFactory.of(6.5222, 102.8766, 90);
    private final Pose point5Control1 = poseFactory.of(10, 40, 0);

    public Path shooting1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path return1() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path pick1() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path shooting2() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path park() {
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    // ---------------- Commands ----------------

    /** Follow a path at DRIVE_SPEED (the limit only applies while that path runs). */
    private Command follow(Path path) {
        return new FollowPathCommand(follower, path.with(
                org.firstinspires.ftc.teamcode.pedro.Constants.foresightConfig.maxPathSpeed.at(DRIVE_SPEED)));
    }

    /** Intake on (0.75) -> drive pick1 -> optional dwell -> intake off. */
    private Command pick1WithIntake() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.setPower(PICK1_INTAKE_POWER), intake),
                follow(pick1()),
                new WaitCommand(PICK1_DWELL_MS),
                new InstantCommand(intake::stop, intake)
        );
    }

    private Command autoRoutine() {
        return new SequentialCommandGroup(
                step("1 shooting1", follow(shooting1())),
                step("2 return1", follow(return1())),
                step("3 pick1 + intake", pick1WithIntake()),
                step("4 shooting2", follow(shooting2())),
                step("5 park", follow(park())),
                step("done", new InstantCommand())
        );
    }

    /** Labels a step so telemetry shows where the routine is. */
    private Command step(String name, Command command) {
        return new SequentialCommandGroup(new InstantCommand(() -> step = name), command);
    }

    // ---------------- OpMode ----------------

    @Override
    protected void initRobot() {
        follower = new FollowerSubsystem(hardwareMap);
        follower.setPose(start);
        intake = new Intake(hardwareMap);
        follower.periodic();
    }

    @Override
    protected void onStart() {
        schedule(autoRoutine());
    }

    @Override
    protected void addTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Step", step);
        telemetry.addData("Follower busy", follower.follower().isBusy());
        telemetry.addData("Intake power", intake.getPower());
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading", pose.heading());
    }
}
