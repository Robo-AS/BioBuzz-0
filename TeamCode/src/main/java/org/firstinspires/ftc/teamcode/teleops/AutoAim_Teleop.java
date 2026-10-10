package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "Auto Aim Test", group = "Test")
public class AutoAim_Teleop extends OpMode {
    // Goal position relative to the robot's start spot (Pedro units, inches).
    private static final double GOAL_X = 70;
    private static final double GOAL_Y = 200;

    private Follower follower;
    private MecanumDrive drive;
    private GamepadEx driver;
    private final PoseFactory p = PoseFactory.degrees();
    Pose startpose = p.of(9, 132, 90);

    @Override
    public void init() {
        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        follower.setPose(startpose); // start at (0, 0), heading 0

        // Create the drive AFTER the follower so your motor directions win.
        drive = new MecanumDrive(hardwareMap);
        drive.setGoal(GOAL_X, GOAL_Y);

        driver = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();

        boolean autoAim = gamepad1.right_trigger > 0.3;

        // Options: re-zero the robot's pose (put the robot on its start spot first)
        if (gamepad1.options) {
            follower.setPose(startpose);
        }

        drive.fieldCentricAutoAim(driver, pose.x(), pose.y(), pose.heading(), autoAim);

        telemetry.addData("Auto aim", autoAim);
        telemetry.addData("Pose", "x=%.1f y=%.1f", pose.x(), pose.y());
        drive.aimTelemetry(telemetry, pose.x(), pose.y(), pose.heading());
        telemetry.update();
    }
}