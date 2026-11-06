package org.firstinspires.ftc.teamcode.autos.RED;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
@Autonomous(name = "🔴 Red Front", group = "🔴 Red")
public class RedFront extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(38.7, 8.4, 90);
    private final Pose returnPose=p.of(38.7,14,90);
    private final Pose park = p.of(8.5, 95, 90);
    private final Pose shooting1 = p.of(56.05, 25.52, 90);
    private final Pose pick1 = p.of(7.6, 7.6, 180);

    private Path shoot()
    {
        return line(startPose, shooting1).linear(startPose, shooting1).with(Constants.foresightConfig.maxPathSpeed.at(0.7));
    }
    private Path pickup()
    {
        return line(startPose, pick1).tangent().with(Constants.foresightConfig.maxPathSpeed.at(0.7));
    }
    private Path returnpickstart()
    {
        return line(pick1, returnPose).linear(pick1, returnPose).with(Constants.foresightConfig.maxPathSpeed.at(0.7));
    }
    private Path returnstart()
    {
        return line(shooting1, returnPose).linear(shooting1, returnPose).with(Constants.foresightConfig.maxPathSpeed.at(0.7));
    }
    private Path park()
    {
        return line(startPose, park).linear(startPose, park).with(Constants.foresightConfig.maxPathSpeed.at(0.7));
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, shoot()),
                waitMs(800),
                follow(follower, returnstart()),
                follow(follower, pickup()),
                follow(follower, returnpickstart()),
                follow(follower, shoot()),
                waitMs(800),
                follow(follower, returnstart()),
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        Pose pose = follower.pose();

        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading", pose.heading());
        telemetry.update();

    }
}