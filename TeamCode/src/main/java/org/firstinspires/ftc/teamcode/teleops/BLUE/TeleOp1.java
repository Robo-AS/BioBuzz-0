package org.firstinspires.ftc.teamcode.teleops.BLUE;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.teleops.BaseTeleOp;

@TeleOp(name = "TeleOp1", group = "Blue")
public class TeleOp1 extends BaseTeleOp {
    private static final PoseFactory P = PoseFactory.degrees();
    private static final Pose START = P.of(132, 9, 90);

    private static final double GOAL_X = 77;
    private static final double GOAL_Y = 71;

    @Override protected Pose startPose() { return START; }
    @Override protected double goalX() { return GOAL_X; }
    @Override protected double goalY() { return GOAL_Y; }
}