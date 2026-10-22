package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.Constants.ROBOT_SPEED;

import java.util.Arrays;
import java.util.List;

public class MecanumDrive {
    public DcMotorEx frontRight, rearLeft, frontLeft, rearRight;
    private List<DcMotorEx> motors;
    double reverse = 1.0;
    public static double powerReduction = 20;

    // ---------------- Auto-aim tuning ----------------
    public static double AIM_KP = 1.6;              // power per radian of error
    public static double AIM_KD = 0.12;             // damping, stops overshoot
    public static double AIM_KS = 0.06;             // minimum power to overcome friction
    public static double AIM_MAX_POWER = 0.8;
    public static double AIM_TOLERANCE_RAD = Math.toRadians(1.0);
    public static double AIM_TURN_SIGN = 1.0;      // flip to +1.0 if it turns AWAY from the goal
    public static double SHOOTER_HEADING_OFFSET_RAD = Math.PI / 2.0; // if shooter isn't on the robot's front

    private double goalX = 0, goalY = 0;
    private double lastAimError = 0;
    private long lastAimTimeNs = 0;
    private boolean aimWasActive = false;
    public boolean aimedAtGoal = false;
    final double velocityLimitTicksPerSecond;
    public static double ACCEL_CHANGE_PER_SECOND = 8.0;
    public static double DECEL_CHANGE_PER_SECOND = 3.0;

    private double profiledX;
    private double profiledY;
    private long lastProfileTimeNs;


    public MecanumDrive(HardwareMap hardwareMap) {
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");

        motors = Arrays.asList(frontRight, rearLeft, frontLeft, rearRight);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        double slowestMotor = Double.POSITIVE_INFINITY;
        for (DcMotorEx motor : motors) {
            slowestMotor = Math.min(
                    slowestMotor,
                    motor.getMotorType().getAchieveableMaxTicksPerSecond()
            );
        }
        velocityLimitTicksPerSecond = slowestMotor * 1;
    }

    public void setDriveVelocityMode(boolean enabled) {
        DcMotorEx.RunMode mode = enabled
                ? DcMotorEx.RunMode.RUN_USING_ENCODER
                : DcMotorEx.RunMode.RUN_WITHOUT_ENCODER;

        for (DcMotorEx motor : motors) {
            motor.setPower(0.0);
            motor.setMode(mode);
        }
    }

    public void fieldCentricVelocity(GamepadEx gamepad, double headingRadians) {
        fieldCentricVelocity(gamepad, headingRadians, addons(-gamepad.getRightX()));
    }

    public void fieldCentricVelocity(GamepadEx gamepad, double headingRadians,
                                     double turnPower) {
        double targetX = addons(gamepad.getLeftY()) * reverse;
        double targetY = addons(-gamepad.getLeftX()) * reverse;

        long now = System.nanoTime();
        double dt = lastProfileTimeNs == 0
                ? 0.0
                : Math.min(0.1, (now - lastProfileTimeNs) / 1_000_000_000.0);
        lastProfileTimeNs = now;

        double changeX = targetX - profiledX;
        double changeY = targetY - profiledY;
        double changeLength = Math.hypot(changeX, changeY);
        double currentSpeed = Math.hypot(profiledX, profiledY);
        double targetAlongMotion = currentSpeed < 0.0001
                ? 0.0
                : (targetX * profiledX + targetY * profiledY) / currentSpeed;

        double changeRate = targetAlongMotion < currentSpeed
                ? DECEL_CHANGE_PER_SECOND
                : ACCEL_CHANGE_PER_SECOND;

        double maximumChange = changeRate * dt;

        if (changeLength > maximumChange && changeLength > 0.0) {
            double scale = maximumChange / changeLength;
            changeX *= scale;
            changeY *= scale;
        }

        profiledX += changeX;
        profiledY += changeY;

        double cos = Math.cos(headingRadians);
        double sin = Math.sin(headingRadians);
        double rotatedX = profiledX * cos + profiledY * sin;
        double rotatedY = -profiledX * sin + profiledY * cos;
        double r = Math.max(-1.0, Math.min(1.0, turnPower));

        double leftFront = rotatedY + rotatedX + r;
        double rightFront = rotatedY - rotatedX - r;
        double leftRear = rotatedY - rotatedX + r;
        double rightRear = rotatedY + rotatedX - r;

        double maximum = Math.max(1.0, Math.max(
                Math.max(Math.abs(leftFront), Math.abs(rightFront)),
                Math.max(Math.abs(leftRear), Math.abs(rightRear))
        ));

        frontLeft.setVelocity(leftFront / maximum * velocityLimitTicksPerSecond);
        frontRight.setVelocity(rightFront / maximum * velocityLimitTicksPerSecond);
        rearLeft.setVelocity(leftRear / maximum * velocityLimitTicksPerSecond);
        rearRight.setVelocity(rightRear / maximum * velocityLimitTicksPerSecond);
    }

    public void resetFreeRoamProfile() {
        profiledX = 0.0;
        profiledY = 0.0;
        lastProfileTimeNs = 0;
    }

    // ---------------- Field centric ----------------
    public void fieldCentric(GamepadEx gamepad, double headingRadians) {
        fieldCentric(gamepad, headingRadians, addons(-gamepad.getRightX()));
    }

    public void fieldCentric(GamepadEx gamepad, double headingRadians, double turnPower) {
        double x = addons(gamepad.getLeftY()) * reverse;
        double y = addons(-gamepad.getLeftX()) * reverse;
        double r = Math.max(-1.0, Math.min(1.0, turnPower));

        double cos = Math.cos(headingRadians);
        double sin = Math.sin(headingRadians);
        double rotatedX = x * cos + y * sin;
        double rotatedY = -x * sin + y * cos;

        double leftFrontPower = rotatedY + rotatedX + r;
        double rightFrontPower = rotatedY - rotatedX - r;
        double leftRearPower = rotatedY - rotatedX + r;
        double rightRearPower = rotatedY + rotatedX - r;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower))));

        frontLeft.setPower(leftFrontPower / maxPower);
        frontRight.setPower(rightFrontPower / maxPower);
        rearLeft.setPower(leftRearPower / maxPower);
        rearRight.setPower(rightRearPower / maxPower);
    }

    /**
     * Field-centric driving. While autoAim is true the driver keeps translation
     * control and the robot holds its heading toward the goal.
     */
    public void fieldCentricAutoAim(GamepadEx gamepad, double x, double y,
                                    double headingRadians, boolean autoAim) {
        if (autoAim) {
            fieldCentric(gamepad, headingRadians, autoAimTurnPower(x, y, headingRadians));
        } else {
            resetAim();
            fieldCentric(gamepad, headingRadians); // normal driver turning
        }
    }

    // ---------------- Auto aim ----------------
    public void setGoal(double x, double y) {
        goalX = x;
        goalY = y;
    }

    public static double angleWrap(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }

    /** Heading (radians, Pedro convention) needed to face the goal from (x, y). */
    public double headingToGoal(double x, double y) {
        return Math.atan2(goalY - y, goalX - x) + SHOOTER_HEADING_OFFSET_RAD;
    }

    /** Turn power that rotates the robot toward the goal. Call every loop while aiming. */
    public double autoAimTurnPower(double x, double y, double headingRadians) {
        double error = angleWrap(headingToGoal(x, y) - headingRadians);

        long now = System.nanoTime();
        double derivative = 0;
        if (aimWasActive && lastAimTimeNs != 0) {
            double dt = (now - lastAimTimeNs) / 1e9;
            if (dt > 1e-4) derivative = angleWrap(error - lastAimError) / dt;
        }
        lastAimError = error;
        lastAimTimeNs = now;
        aimWasActive = true;

        aimedAtGoal = Math.abs(error) < AIM_TOLERANCE_RAD;
        if (aimedAtGoal) return 0;

        double out = AIM_KP * error + AIM_KD * derivative;
        out += Math.signum(error) * AIM_KS;
        out = Math.max(-AIM_MAX_POWER, Math.min(AIM_MAX_POWER, out));
        return out * AIM_TURN_SIGN;
    }

    /** Called when auto-aim is released so the D term doesn't use stale data. */
    public void resetAim() {
        aimWasActive = false;
        aimedAtGoal = false;
    }

    // ---------------- Robot centric ----------------
    public void teleop(GamepadEx gamepad, Telemetry telemetry) {
        double x = -gamepad.getLeftX();
        double y = -gamepad.getLeftY();
        double r = -gamepad.getRightX();

        x = addons(x) * reverse;
        y = addons(y) * reverse;
        r = addons(r);

        double LeftFrontPower = (y + x + r);
        double RightFrontPower = (y - x - r);
        double LeftRearPower = (y - x + r);
        double RightRearPower = (y + x - r);

        frontLeft.setPower(LeftFrontPower);
        frontRight.setPower(RightFrontPower);
        rearLeft.setPower(LeftRearPower);
        rearRight.setPower(RightRearPower);
    }

    // ---------------- Helpers ----------------
    public double addons(double value) {
        if (Math.abs(value) < CONTROLLER_DEADZONE) return 0;
        return value * ROBOT_SPEED;
    }

    double clip(double power) {
        return Math.max(-1.0, Math.min(power, 1.0));
    }

    // ---------------- Telemetry ----------------
    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("---ROBOT CENTRIC DRIVE---");

        telemetry.addData("Direction Multiplier: ", reverse);
        telemetry.addData("Speed Multiplier: ", ROBOT_SPEED);

        telemetry.addData("LeftRear Position: ", rearLeft.getCurrentPosition());
        telemetry.addData("RightRear Position: ", rearRight.getCurrentPosition());
        telemetry.addData("LeftFront Position: ", frontLeft.getCurrentPosition());
        telemetry.addData("RightFront Position: ", frontRight.getCurrentPosition());

        telemetry.addData("LeftRear Power: ", rearLeft.getPower());
        telemetry.addData("RightRear Power: ", rearRight.getPower());
        telemetry.addData("LeftFront Power: ", frontLeft.getPower());
        telemetry.addData("RightFront Power: ", frontRight.getPower());
    }

    public void aimTelemetry(Telemetry telemetry, double x, double y, double headingRadians) {
        telemetry.addLine("---AUTO AIM---");
        telemetry.addData("Goal", "%.1f, %.1f", goalX, goalY);
        telemetry.addData("Target heading (deg)", Math.toDegrees(headingToGoal(x, y)));
        telemetry.addData("Current heading (deg)", Math.toDegrees(headingRadians));
        telemetry.addData("Error (deg)",
                Math.toDegrees(angleWrap(headingToGoal(x, y) - headingRadians)));
        telemetry.addData("Aimed", aimedAtGoal);
    }
}