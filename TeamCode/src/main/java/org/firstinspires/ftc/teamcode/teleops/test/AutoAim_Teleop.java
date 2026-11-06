package org.firstinspires.ftc.teamcode.teleops.test;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleops.BaseTeleOp;

@TeleOp(name = "Auto Aim Test", group = "BLUE")
public class AutoAim_Teleop extends BaseTeleOp {
    private static final PoseFactory P = PoseFactory.degrees();
    private static final Pose START = P.of(138, 7, 90);

    private static final double GOAL_X = 72;
    private static final double GOAL_Y = 72;

    @Override protected Pose startPose() { return START; }
    @Override protected double goalX() { return GOAL_X; }
    @Override protected double goalY() { return GOAL_Y; }
}
