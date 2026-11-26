package org.firstinspires.ftc.teamcode.subs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * "faras" continuous rotation servo, moved between two angles using its feedback wire
 * (analog input "farasEncoder"). Same positions as the Servo Tuner:
 *   where it is at START = 56°, toggle -> 380°, toggle -> 56°, ...
 *
 * Until the first toggle it gets NO power (same as the tuner's start).
 *
 * LIVE TUNING in FTC Dashboard (http://192.168.43.1:8080/dash):
 *   Config panel -> "Faras PID": KP, KD, KS, MAX_POWER, TOLERANCE_DEG, REENGAGE_DEG, VELOCITY_FILTER.
 *     Change a value, press Enter / Save -> used on the very next loop. (Press ↻ if it doesn't show.)
 *   Graph panel: "Faras target", "Faras angle", "Faras power x100".
 *   Values reset on redeploy: copy good ones into the defaults below.
 *
 * Control = PD + static feedforward instead of the tuner's P-only loop, to stop the oscillation:
 *   P  pushes toward the target, D brakes using the measured speed (kills overshoot),
 *   kS just beats the servo's deadband, and a small hold band with hysteresis stops it
 *   from twitching back and forth once it has arrived.
 */
public class FarasSubsystem extends SubsystemBase {

    // ---- Positions (angles from where the servo is at START) ----
    public static double ANGLE_A = 56;   // start position
    public static double ANGLE_B = 380;

    // ---- PD tuning (live-editable in FTC Dashboard -> Config -> "Faras PID") ----
    public static volatile double KP = 0.008;      // power per degree of error
    public static volatile double KD = 0.0005;     // power per degree/second of speed (braking)
    public static volatile double KS = 0.04;       // minimum push to overcome the deadband
    public static volatile double MAX_POWER = 1.0; // full speed
    public static volatile double TOLERANCE_DEG = 3.0;  // "arrived" band -> power 0
    public static volatile double REENGAGE_DEG = 6.0;   // only start correcting again if it drifts this far
    public static volatile double VELOCITY_FILTER = 0.3; // 0..1, higher = less smoothing of the speed estimate
    public static boolean INVERT_FEEDBACK = false;

    private final CRServo crServo;
    private final Servo servo;
    private final AnalogInput feedback;

    private boolean active = false;   // false = no power (limp) until the first toggle
    private boolean atB = false;
    private boolean settled = false;
    private boolean pwmOn = true;

    private double lastRaw = Double.NaN;
    private double travelled = 0;
    private double angle = ANGLE_A;
    private double velocity = 0;      // deg/s, filtered
    private double power = 0;
    private long lastNs = 0;

    public FarasSubsystem(HardwareMap hardwareMap) {
        // Works whether "faras" is configured as a Continuous Rotation Servo or a plain Servo.
        crServo = hardwareMap.tryGet(CRServo.class, "faras");
        servo = crServo == null ? hardwareMap.get(Servo.class, "faras") : null;
        feedback = hardwareMap.get(AnalogInput.class, "farasEncoder");
        registerDashboardConfig();
    }

    // ---------------- FTC Dashboard live config ----------------
    // Registered by the code itself (not @Config): on FTC SDK 12.0 @Config values saved in the
    // Dashboard don't reach the running code (ftc-dashboard issue #240). Same fix as the Flywheel Tuner.
    private static final String CATEGORY = "Faras PID";

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
        addVar(dash, "KP", () -> KP, v -> KP = Math.max(0, v));
        addVar(dash, "KD", () -> KD, v -> KD = Math.max(0, v));
        addVar(dash, "KS", () -> KS, v -> KS = Math.max(0, v));
        addVar(dash, "MAX_POWER", () -> MAX_POWER, v -> MAX_POWER = Math.max(0, Math.min(1, v)));
        addVar(dash, "TOLERANCE_DEG", () -> TOLERANCE_DEG, v -> TOLERANCE_DEG = Math.max(0, v));
        addVar(dash, "REENGAGE_DEG", () -> REENGAGE_DEG, v -> REENGAGE_DEG = Math.max(0, v));
        addVar(dash, "VELOCITY_FILTER", () -> VELOCITY_FILTER, v -> VELOCITY_FILTER = Math.max(0.01, Math.min(1, v)));
        dash.updateConfig();
    }

    // ---------------- Actions ----------------

    /** Go to the other angle (56 <-> 380). The first call goes to 380. */
    public void toggle() {
        atB = !atB;
        active = true;
        settled = false;
    }

    public Command toggleCommand() {
        return new InstantCommand(this::toggle, this);
    }

    public double target() { return atB ? ANGLE_B : ANGLE_A; }
    public double angle() { return angle; }
    public boolean atTarget() { return active && Math.abs(target() - angle) <= TOLERANCE_DEG; }

    // ---------------- Control loop ----------------

    @Override
    public void periodic() {
        long now = System.nanoTime();
        double dt = lastNs == 0 ? 0 : (now - lastNs) / 1e9;
        lastNs = now;

        // Read + unwrap the feedback (continuous across 360°/0°). First reading = ANGLE_A.
        double raw = feedback.getVoltage() / feedback.getMaxVoltage() * 360.0;
        if (INVERT_FEEDBACK) raw = 360.0 - raw;
        double delta = 0;
        if (!Double.isNaN(lastRaw)) {
            delta = raw - lastRaw;
            if (delta > 180) delta -= 360;
            if (delta < -180) delta += 360;
            travelled += delta;
        }
        lastRaw = raw;
        angle = ANGLE_A + travelled;

        if (dt > 0) {
            double rawVelocity = delta / dt;
            velocity += VELOCITY_FILTER * (rawVelocity - velocity); // low-pass: the analog signal is noisy
        }

        if (!active) {
            power = 0;
            setLimp(true);
            sendGraph();
            return;
        }
        setLimp(false);

        double error = target() - angle;
        if (settled && Math.abs(error) < REENGAGE_DEG) {
            power = 0;                                  // arrived: don't twitch
        } else if (Math.abs(error) <= TOLERANCE_DEG) {
            power = 0;
            settled = true;
        } else {
            settled = false;
            double out = KP * error - KD * velocity + Math.signum(error) * KS;
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, out));
        }

        if (crServo != null) crServo.setPower(power);
        else servo.setPosition(0.5 + power / 2.0); // plain-Servo config: 0.5 = stop

        sendGraph();
    }

    /** Live graph in FTC Dashboard: target vs. actual angle, plus power. */
    private void sendGraph() {
        FtcDashboard dash = FtcDashboard.getInstance();
        if (dash == null) return;
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Faras target", active ? target() : angle);
        packet.put("Faras angle", angle);
        packet.put("Faras power x100", power * 100);
        dash.sendTelemetryPacket(packet);
    }

    private void setLimp(boolean limp) {
        if (limp == !pwmOn) return;
        if (limp) {
            if (crServo instanceof CRServoImplEx) ((CRServoImplEx) crServo).setPwmDisable();
            else if (crServo != null) crServo.setPower(0);
            if (servo instanceof ServoImplEx) ((ServoImplEx) servo).setPwmDisable();
        } else {
            if (crServo instanceof CRServoImplEx) ((CRServoImplEx) crServo).setPwmEnable();
            if (servo instanceof ServoImplEx) ((ServoImplEx) servo).setPwmEnable();
        }
        pwmOn = !limp;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Faras", !active ? "no power (dpad up = go to " + (int) ANGLE_B + "°)"
                : String.format("target %.0f°  angle %.1f°  error %.1f°%s",
                target(), angle, target() - angle, atTarget() ? "  ✓" : ""));
        telemetry.addData("Faras power / speed", "%.2f / %.0f°/s", power, velocity);
    }
}
