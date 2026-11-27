package org.firstinspires.ftc.teamcode.autos.BLUE;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subs.BlockerServo;
import org.firstinspires.ftc.teamcode.subs.FarasSubsystem;
import org.firstinspires.ftc.teamcode.subs.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subs.HoodServo;
import org.firstinspires.ftc.teamcode.subs.Intake;
import org.firstinspires.ftc.teamcode.subs.IntakeServo;
import org.firstinspires.ftc.teamcode.subs.ShooterSubsystem;

@Autonomous(name = "🔵 Blue Test - Full Auto", group = "🔵 Blue")
public class BlueTest extends RobotOpMode {

    public static double INTAKE_POWER = 0.75;
    public static double DRIVE_SPEED = 0.6;
    public static long FLYWHEEL_SPINUP_MS = 1000;
    public static long SHOOT_WAIT_MS = 1200;
    public static double HOOD_POSITION = 0.47;

    private FollowerSubsystem follower;
    private Intake intake;
    private ShooterSubsystem shooter;
    private BlockerServo blocker;
    private IntakeServo intakeServo;
    private FarasSubsystem faras;
    private HoodServo hood;

    private String step = "waiting for START";

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(87.9039, 131.6771, 90);
    private final Pose path1Pose = poseFactory.of(87.7063, 113.9454, 90);
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

    public Path path1() {
        return Paths.line(start, path1Pose).constant(path1Pose);
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

    private Command follow(Path path) {
        return new FollowPathCommand(follower, path.with(
                org.firstinspires.ftc.teamcode.pedro.Constants.foresightConfig.maxPathSpeed.at(DRIVE_SPEED)));
    }

    /** Sequence to shoot elements: set hood -> start flywheel -> WAIT FOR SPINUP -> unblock -> feeder on -> wait -> stop shooter. */
    private Command shootSequence() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> hood.setPosition(HOOD_POSITION), hood),
                new InstantCommand(() -> shooter.setFlywheel(true), shooter),
                new WaitCommand(FLYWHEEL_SPINUP_MS), // Așteptăm ca flywheel-ul să prindă viteză
                new InstantCommand(blocker::notblocked, blocker),
                new InstantCommand(intakeServo::toB, intakeServo),
                new WaitCommand(300),
                new InstantCommand(() -> shooter.setFeeder(true), shooter),
                new InstantCommand(() -> intake.setPower(1.0), intake),
                new WaitCommand(SHOOT_WAIT_MS),
                new InstantCommand(shooter::stopAll, shooter),
                new InstantCommand(blocker::blocked, blocker),
                new InstantCommand(intakeServo::toA, intakeServo),
                new InstantCommand(intake::stop, intake)
        );
    }

    /** Intake on -> drive path -> wait -> intake off. */
    private Command pickWithIntake(Path path, long dwellMs) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.setPower(INTAKE_POWER), intake),
                new InstantCommand(intakeServo::toB, intakeServo),
                follow(path),
                new WaitCommand(dwellMs),
                new InstantCommand(intake::stop, intake),
                new InstantCommand(intakeServo::toA, intakeServo)
        );
    }

    public Command autoRoutine() {
        return new SequentialCommandGroup(
                step("1 path1 + shoot", new SequentialCommandGroup(follow(path1()), shootSequence())),
                step("2 path2", follow(path2())),
                step("3 path3 pick", pickWithIntake(path3(), 300)),
                step("4 path4 (83,20) + shoot + faras down", new SequentialCommandGroup(
                        follow(path4()),
                        shootSequence(),
                        new InstantCommand(faras::toggle, faras),
                        new WaitCommand(500)
                )),
                step("5 path5 (94,17) + faras up", new SequentialCommandGroup(
                        follow(path5()),
                        new InstantCommand(faras::toggle, faras),
                        new WaitCommand(500)
                )),
                step("6 path6 (83,20)", follow(path6())),
                step("7 path7 (122,94)", follow(path7())),
                step("8 path8 + shoot", new SequentialCommandGroup(follow(path8()), shootSequence())),
                step("9 path9 park", follow(path9())),
                step("done", new InstantCommand())
        );
    }

    private Command step(String name, Command command) {
        return new SequentialCommandGroup(new InstantCommand(() -> step = name), command);
    }

    @Override
    protected void initRobot() {
        follower = new FollowerSubsystem(hardwareMap);
        follower.setPose(start);

        intake = new Intake(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        blocker = new BlockerServo(hardwareMap);
        intakeServo = new IntakeServo(hardwareMap);
        faras = new FarasSubsystem(hardwareMap);
        hood = new HoodServo(hardwareMap);

        blocker.blocked();
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
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading", pose.heading());
    }
}