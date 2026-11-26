package org.firstinspires.ftc.teamcode.subs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * "faras" continuous rotation servo, TIME-BASED (no encoder needed).
 *
 *   The faras must be UP when the OpMode starts. After START it HOLDS UP.
 *   toggle() -> runs DOWN for DOWN_TIME_MS, then HOLDS DOWN
 *   toggle() -> runs UP   for UP_TIME_MS,   then HOLDS UP
 *   Toggling in the middle of a move reverses it for the time already spent, so it returns to where it was.
 *
 * Holding: a small constant power keeps it pushed UP (against gravity) or DOWN so it doesn't sag.
 * Keep the hold powers LOW: a continuous servo pushing against something all match long heats up.
 *
 * All values are live-tunable in FTC Dashboard -> Config -> "Faras".
 * If toggle() makes it go UP instead of DOWN, set DOWN_SIGN = -1.
 */
public class FarasSubsystem extends SubsystemBase {

    /** +1 if positive power moves the faras DOWN, -1 if it moves it UP. */
    public static volatile double DOWN_SIGN = -1;

    public static volatile double MOVE_POWER = 0.7;       // speed while moving up or down
    public static volatile double DOWN_TIME_MS = 500;     // how long to run DOWN
    public static volatile double UP_TIME_MS = 500;       // how long to run UP (against gravity, may need longer)
    public static volatile double HOLD_UP_POWER = 0.1;    // keeps it pushed UP at the top
    public static volatile double HOLD_DOWN_POWER = 0.05; // keeps it pushed DOWN at the bottom

    private final CRServo crServo;
    private final Servo servo;

    private boolean down = false;         // where it is / is going
    private boolean moving = false;
    private long moveStartNs = 0;
    private double moveDurationMs = 0;
    private double power = 0;

    public FarasSubsystem(HardwareMap hardwareMap) {
        // Works whether "faras" is configured as a Continuous Rotation Servo or a plain Servo.
        crServo = hardwareMap.tryGet(CRServo.class, "faras");
        servo = crServo == null ? hardwareMap.get(Servo.class, "faras") : null;

        // Axon signal range is 500-2500 us (set in the Axon app); the hub default is narrower.
        PwmControl.PwmRange axonRange = new PwmControl.PwmRange(500, 2500);
        if (crServo instanceof CRServoImplEx) ((CRServoImplEx) crServo).setPwmRange(axonRange);
        if (servo instanceof ServoImplEx) ((ServoImplEx) servo).setPwmRange(axonRange);

        registerDashboardConfig();
    }

    // ---------------- Actions ----------------

    /** UP -> DOWN -> UP ... (first call goes DOWN). */
    public void toggle() {
        long now = System.nanoTime();
        double spentMs = moving ? (now - moveStartNs) / 1e6 : 0;
        down = !down;
        moveDurationMs = moving ? spentMs : (down ? DOWN_TIME_MS : UP_TIME_MS); // mid-move: go back
        moveStartNs = now;
        moving = true;
    }

    public Command toggleCommand() {
        return new InstantCommand(this::toggle, this);
    }

    /** Add `ms` to the time of the CURRENT direction (DOWN time if it's down, UP time if it's up). */
    public void trimTime(double ms) {
        if (down) DOWN_TIME_MS = Math.max(0, DOWN_TIME_MS + ms);
        else UP_TIME_MS = Math.max(0, UP_TIME_MS + ms);
    }

    public boolean isDown() { return down; }
    public boolean isMoving() { return moving; }

    // ---------------- Loop ----------------

    @Override
    public void periodic() {
        if (moving && (System.nanoTime() - moveStartNs) / 1e6 >= moveDurationMs) moving = false;

        if (moving) {
            power = (down ? 1 : -1) * DOWN_SIGN * MOVE_POWER;
        } else {
            power = down ? DOWN_SIGN * HOLD_DOWN_POWER : -DOWN_SIGN * HOLD_UP_POWER;
        }

        if (crServo != null) crServo.setPower(power);
        else servo.setPosition(0.5 + power / 2.0); // plain-Servo config: 0.5 = stop
    }

    // ---------------- Telemetry / Dashboard ----------------

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Faras", "%s%s   power %.2f",
                down ? "DOWN" : "UP", moving ? " (moving)" : " (holding)", power);
        telemetry.addData("Faras times", "DOWN %.0f ms  /  UP %.0f ms", DOWN_TIME_MS, UP_TIME_MS);
    }

    // Registered by the code itself (not @Config): @Config doesn't work on FTC SDK 12.0 (issue #240).
    private static final String CATEGORY = "Faras";

    private static void addVar(FtcDashboard dash, String name, Supplier<Double> get, Consumer<Double> set) {
        try { dash.removeConfigVariable(CATEGORY, name); } catch (Exception ignored) { }
        dash.addConfigVariable(CATEGORY, name, new ValueProvider<Double>() {
            @Override public Double get() { return get.get(); }
            @Override public void set(Double value) { if (value != null) set.accept(value); }
        });
    }

    private void registerDashboardConfig() {
        FtcDashboard dash = FtcDashboard.getInstance();
        if (dash == null) return;
        for (String old : new String[]{"MAX_POWER", "MIN_POWER", "SLOW_ZONE_DEG", "TOLERANCE_DEG", "REENGAGE_DEG", "STALL_TIME_S"}) {
            try { dash.removeConfigVariable(CATEGORY, old); } catch (Exception ignored) { }
        }
        addVar(dash, "DOWN_SIGN", () -> DOWN_SIGN, v -> DOWN_SIGN = v >= 0 ? 1 : -1);
        addVar(dash, "MOVE_POWER", () -> MOVE_POWER, v -> MOVE_POWER = Math.max(0, Math.min(1, v)));
        addVar(dash, "DOWN_TIME_MS", () -> DOWN_TIME_MS, v -> DOWN_TIME_MS = Math.max(0, v));
        addVar(dash, "UP_TIME_MS", () -> UP_TIME_MS, v -> UP_TIME_MS = Math.max(0, v));
        addVar(dash, "HOLD_UP_POWER", () -> HOLD_UP_POWER, v -> HOLD_UP_POWER = Math.max(0, Math.min(1, v)));
        addVar(dash, "HOLD_DOWN_POWER", () -> HOLD_DOWN_POWER, v -> HOLD_DOWN_POWER = Math.max(0, Math.min(1, v)));
        dash.updateConfig();
    }
}
