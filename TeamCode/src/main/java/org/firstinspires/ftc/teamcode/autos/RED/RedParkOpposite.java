package org.firstinspires.ftc.teamcode.autos.RED;

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
import org.firstinspires.ftc.teamcode.subs.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subs.HoodServo;
import org.firstinspires.ftc.teamcode.subs.Intake;
import org.firstinspires.ftc.teamcode.subs.IntakeServo;
import org.firstinspires.ftc.teamcode.subs.ShooterSubsystem;

@Autonomous(name = "Red Park Opoosite", group = "Red")
public class RedParkOpposite extends RobotOpMode {

    public static double INTAKE_POWER = 1.0;
    public static double DRIVE_SPEED = 1;
    public static long FLYWHEEL_WAIT_MS = 2500; // Așteaptă 2.5 secunde ca flywheel-ul să ajungă la turație maximă
    public static long SHOOT_WAIT_MS = 2200;
    public static double HOOD_POSITION = 0.47;
    public static double FLYWHEEL_RPM = 3790;

    private FollowerSubsystem follower;
    private Intake intake;
    private ShooterSubsystem shooter;
    private BlockerServo blocker;
    private IntakeServo intakeServo;
    private HoodServo hood;

    private String step = "waiting for START";

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.6, 132.3827, 90);
    private final Pose path1 = poseFactory.of(7.0251, 121.1452, 90);
    private final Pose path1Control1 = poseFactory.of(51.7657, 93.185, 0);
    public Path path1() {
        return Paths.curve(start, path1Control1, path1).constant(path1);
    }


    private Command follow(Path path) {
        return new FollowPathCommand(follower, path.with(
                org.firstinspires.ftc.teamcode.pedro.Constants.foresightConfig.maxPathSpeed.at(DRIVE_SPEED)));
    }

    /** Sequence to shoot elements: feeder on -> wait -> stop feeder/intake. */
    private Command shootSequence() {
        return new SequentialCommandGroup(
                new InstantCommand(intakeServo::toB, intakeServo),
                new InstantCommand(blocker::notblocked, blocker),
                new WaitCommand(1500),
                new InstantCommand(() -> shooter.setFeeder(true), shooter),
                new InstantCommand(() -> intake.setPower(1.0), intake),
                new WaitCommand(SHOOT_WAIT_MS),
                new InstantCommand(() -> shooter.setFeeder(false), shooter),
                new InstantCommand(blocker::blocked, blocker),
                new InstantCommand(intakeServo::toA, intakeServo),
                new InstantCommand(intake::stop, intake)
        );
    }

    public Command autoRoutine() {
        return new SequentialCommandGroup(
                // Pornim flywheel-ul și setăm hood-ul din prima secundă în timp ce robotul se deplasează
                new InstantCommand(() -> hood.setPosition(HOOD_POSITION), hood),
                new InstantCommand(() -> shooter.setTargetRpm(FLYWHEEL_RPM), shooter),
                new InstantCommand(() -> shooter.setFlywheel(true), shooter),
                new WaitCommand(10000),
                step("2 shoot", shootSequence()),
                step("go path1 ", follow(path1())),
                step("done", new InstantCommand(shooter::stopAll, shooter))
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
        hood = new HoodServo(hardwareMap);

        // Blocker-ul rămâne deschis (notblocked) tot timpul
        blocker.notblocked();
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