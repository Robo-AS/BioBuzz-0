package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "Auto Aim Test", group = "BLUE")
public class AutoAim_Teleop extends OpMode {
    // Goal position relative to the robot's start spot (Pedro units, inches).
    private static final double GOAL_X = 72;
    private static final double GOAL_Y = 72;

    private Follower follower;
    private MecanumDrive drive;
    private GamepadEx driver;
    private final PoseFactory p = PoseFactory.degrees();
    Pose startpose = p.of(138, 7, 90);
    private boolean previousAutoAim = true;
    private static final double HOLD_KP = 0.8;
    private static final double MAX_HOLD_TURN = 0.25;
    private static final double TURN_STICK_DEADZONE = 0.08;

    private double heldHeading;
    private boolean headingHoldReady;

    private static final long HEADING_SETTLE_NS = 200_000_000L;
    private long headingSettleStartNs;
    private boolean wasManualTurn;

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
            pose = follower.pose();
            headingHoldReady = false;
            headingSettleStartNs = System.nanoTime();
            drive.resetFreeRoamProfile();
        }

        boolean justReleasedAutoAim = !autoAim && previousAutoAim;

        if (autoAim != previousAutoAim) {
            drive.setDriveVelocityMode(!autoAim);
            drive.resetFreeRoamProfile();
            previousAutoAim = autoAim;
        }

        if (autoAim) {
            drive.fieldCentricAutoAim(driver, pose.x(), pose.y(), pose.heading(), true);
        } else {
            drive.resetAim();

            double stickTurn = -driver.getRightX();
            boolean manualTurn = Math.abs(stickTurn) > TURN_STICK_DEADZONE;
            long now = System.nanoTime();
            double turnPower;

            if (manualTurn) {
                heldHeading = pose.heading();
                headingHoldReady = false;
                turnPower = drive.addons(stickTurn);
            } else {
                if (wasManualTurn || justReleasedAutoAim) {
                    headingSettleStartNs = now;
                    headingHoldReady = false;
                }

                if (!headingHoldReady) {
                    heldHeading = pose.heading();
                    turnPower = 0.0;

                    if (now - headingSettleStartNs >= HEADING_SETTLE_NS) {
                        headingHoldReady = true;
                    }
                } else {
                    double error = MecanumDrive.angleWrap(heldHeading - pose.heading());
                    turnPower = HOLD_KP * error * MecanumDrive.AIM_TURN_SIGN;
                    turnPower = Math.max(-MAX_HOLD_TURN,
                            Math.min(MAX_HOLD_TURN, turnPower));

                    if (Math.abs(error) < Math.toRadians(1.0)) {
                        turnPower = 0.0;
                    }
                }
            }

            wasManualTurn = manualTurn;
            double headingError = MecanumDrive.angleWrap(heldHeading - pose.heading());

            drive.fieldCentricVelocity(driver, pose.heading(), turnPower);

            telemetry.addData("Hold heading (deg)", Math.toDegrees(heldHeading));
            telemetry.addData("Hold error (deg)", Math.toDegrees(headingError));
    }

        double fl = Math.abs(drive.frontLeft.getVelocity());
        double fr = Math.abs(drive.frontRight.getVelocity());
        double rl = Math.abs(drive.rearLeft.getVelocity());
        double rr = Math.abs(drive.rearRight.getVelocity());

        telemetry.addData("Follower mode", follower.mode());

        telemetry.addData("FL mode", drive.frontLeft.getMode());
        telemetry.addData("FR mode", drive.frontRight.getMode());
        telemetry.addData("RL mode", drive.rearLeft.getMode());
        telemetry.addData("RR mode", drive.rearRight.getMode());
        telemetry.addData("FL speed (ticks/s)", "%.0f", fl);
        telemetry.addData("FR speed (ticks/s)", "%.0f", fr);
        telemetry.addData("RL speed (ticks/s)", "%.0f", rl);
        telemetry.addData("RR speed (ticks/s)", "%.0f", rr);

        telemetry.addData("Auto aim", autoAim);
        telemetry.addData("Pose", "x=%.1f y=%.1f", pose.x(), pose.y());
        drive.aimTelemetry(telemetry, pose.x(), pose.y(), pose.heading());
        telemetry.update();
    }
}