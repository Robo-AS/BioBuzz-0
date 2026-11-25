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
import org.firstinspires.ftc.teamcode.subs.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subs.Intake;
import org.firstinspires.ftc.teamcode.subs.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subs.SweeperSubsystem;

@Autonomous(name = "🔵 Blue Front - Full Auto", group = "🔵 Blue")
public class BlueBack extends RobotOpMode {

    public static double INTAKE_POWER = 0.75;
    public static double DRIVE_SPEED = 0.7;

    // Setări pentru Sweeper (CRServo)
    public static double SWEEPER_DOWN_POWER = 0.8;
    public static double SWEEPER_UP_POWER = -0.8;
    public static long SWEEPER_TIME_MS = 350;

    // Timpi de așteptare
    public static long SPINUP_WAIT_MS = 300;
    public static long FEEDER_SHOOT_MS = 1200;
    public static long PICK_WAIT_MS = 500;
    public static long FLOWER_WAIT_MS = 800;

    private FollowerSubsystem follower;
    private Intake intake;
    private ShooterSubsystem shooter;
    private SweeperSubsystem sweeper;
    private String step = "waiting for START";

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poziții BLUE
    private final Pose startPose = poseFactory.of(103.0, 134.0, 90);
    private final Pose shootingPose = poseFactory.of(84.0, 121.0, 90);
    private final Pose returnPose = poseFactory.of(103.0, 134.0, 90);
    private final Pose pickPose = poseFactory.of(132.8, 134.3, 0);
    private final Pose flower1Pose = poseFactory.of(113.0, 92.0, 0);
    private final Pose flower2Pose = poseFactory.of(130.0, 93.0, 0);
    private final Pose parkPose = poseFactory.of(131.0, 46.0, 90);

    // ---------------- Paths ----------------

    public Path shoot1() {
        return Paths.line(startPose, shootingPose).linear(startPose, shootingPose);
    }

    public Path returnFromShoot() {
        return Paths.line(shootingPose, returnPose).linear(shootingPose, returnPose);
    }

    public Path goToPick() {
        return Paths.line(returnPose, pickPose).tangent();
    }

    public Path shoot2() {
        return Paths.line(pickPose, shootingPose).linear(pickPose, shootingPose);
    }

    public Path alignFlower() {
        return Paths.line(shootingPose, flower1Pose).linear(shootingPose, flower1Pose);
    }

    public Path enterFlower() {
        return Paths.line(flower1Pose, flower2Pose).tangent();
    }

    public Path shoot3() {
        return Paths.line(flower2Pose, shootingPose).linear(flower2Pose, shootingPose);
    }

    public Path clearShooting() {
        return Paths.line(shootingPose, returnPose).linear(shootingPose, returnPose);
    }

    public Path park() {
        return Paths.line(returnPose, parkPose).linear(returnPose, parkPose);
    }

    // ---------------- Commands ----------------

    private Command follow(Path path) {
        return new FollowPathCommand(follower, path.with(
                org.firstinspires.ftc.teamcode.pedro.Constants.foresightConfig.maxPathSpeed.at(DRIVE_SPEED)));
    }

    private Command pickWithIntake(Path path, long dwellMs) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.setPower(INTAKE_POWER), intake),
                follow(path),
                new WaitCommand(dwellMs),
                new InstantCommand(intake::stop, intake)
        );
    }

    private Command driveAndShoot(Path pathToShoot) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setFlywheel(true), shooter),
                follow(pathToShoot),
                new WaitCommand(SPINUP_WAIT_MS),
                new InstantCommand(() -> shooter.setFeeder(true), shooter),
                new WaitCommand(FEEDER_SHOOT_MS),
                new InstantCommand(shooter::stopAll, shooter)
        );
    }

    private Command collectFromFlower() {
        return new SequentialCommandGroup(
                // 1. Pornește intake-ul și intră în poziția flower2
                new InstantCommand(() -> intake.setPower(INTAKE_POWER), intake),
                follow(enterFlower()),

                // 2. Coboară sweeper-ul și oprește CRServo-ul
                //new InstantCommand(() -> sweeper.setPower(SWEEPER_DOWN_POWER), sweeper),
                new WaitCommand(SWEEPER_TIME_MS),
                new InstantCommand(sweeper::stop, sweeper),

                // 3. Așteaptă ca intake-ul să tragă mingile din floare
                new WaitCommand(FLOWER_WAIT_MS),

                // 4. Ridică sweeper-ul la loc și oprește tot
                //new InstantCommand(() -> sweeper.setPower(SWEEPER_UP_POWER), sweeper),
                new WaitCommand(SWEEPER_TIME_MS),
                new InstantCommand(sweeper::stop, sweeper),
                new InstantCommand(intake::stop, intake)
        );
    }

    private Command autoRoutine() {
        return new SequentialCommandGroup(
                step("1 shoot1", driveAndShoot(shoot1())),
                step("2 return", follow(returnFromShoot())),
                step("3 pick + intake", pickWithIntake(goToPick(), PICK_WAIT_MS)),
                step("4 shoot2", driveAndShoot(shoot2())),
                step("5 align flower", follow(alignFlower())),
                step("6 collect flower + sweeper", collectFromFlower()),
                step("7 shoot3", driveAndShoot(shoot3())),
                step("8 return", follow(clearShooting())),
                step("9 park", follow(park())),
                step("done", new InstantCommand())
        );
    }

    private Command step(String name, Command command) {
        return new SequentialCommandGroup(new InstantCommand(() -> step = name), command);
    }

    // ---------------- OpMode ----------------

    @Override
    protected void initRobot() {
        follower = new FollowerSubsystem(hardwareMap);
        follower.setPose(startPose);
        intake = new Intake(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        sweeper = new SweeperSubsystem(hardwareMap);
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