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
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
@Autonomous
public class StartParking extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(38.7, 8.6, 90);
    private final Pose park = p.of(9.57, 110.99, 90);
    private final Pose shooting1 = p.of(56.05, 25.52, 90);
    private final Pose pick1 = p.of(7.6, 7.6, 180);

    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }
    private Path shoot()
    {
        return line(startPose, shooting1).linear(startPose, shooting1);
    }
    private Path pickup()
    {
        return line(startPose, pick1).tangent();
    }
    private Path returnpickstart()
    {
        return line(pick1, startPose).linear(pick1, startPose);
    }
    private Path returnstart()
    {
        return line(shooting1, startPose).linear(shooting1, startPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, shoot()),
                follow(follower, returnstart()),
                follow(follower, pickup()),
                follow(follower, returnpickstart()),
                follow(follower, shoot()),
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