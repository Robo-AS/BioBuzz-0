package org.firstinspires.ftc.teamcode.teleops.RED;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleops.BaseTeleOp;

@TeleOp(name = "TeleOp2", group = "Test")
public class TeleOp2 extends BaseTeleOp {
    private static final PoseFactory P = PoseFactory.degrees();
    // TODO check: same start pose as BLUE TeleOp1 — probably needs the real RED start spot.
    private static final Pose START = P.of(9, 132, 90);

    // Goal position (Pedro units, inches).
    // TODO check: y = 208 is outside the 144" field.
    private static final double GOAL_X = 58;
    private static final double GOAL_Y = 71;

    @Override protected Pose startPose() { return START; }
    @Override protected double goalX() { return GOAL_X; }
    @Override protected double goalY() { return GOAL_Y; }
}
