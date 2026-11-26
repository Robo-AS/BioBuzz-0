package org.firstinspires.ftc.teamcode.shootertest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subs.Intake;
import org.firstinspires.ftc.teamcode.subs.ShooterSubsystem;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/*
 * TEST TOOL — flywheel speed + PIDF tuner, with live graphs in FTC Dashboard.
 *
 * FTC Dashboard: connect to the robot Wi-Fi, open http://192.168.43.1:8080/dash
 *   Graph panel: tick "Target RPM" and "Actual RPM" (and "Error RPM", "Power x1000").
 *   Config panel -> "Flywheel Tuner": edit flywheelOn/intakeOn/feederOn/kP/kI/kD/kF/targetRpm live,
 *   then press Save.
 *   (Only appears while this OpMode is INIT/running. Press the ↻ refresh icon if it doesn't show.)
 *
 * Controls (gamepad1, one step per press) — they change the SAME values as the Dashboard:
 *   A               flywheel on / off (same as ticking flywheelOn in the Dashboard Config tab)
 *   dpad up/down    target speed +/- 20 RPM
 *   dpad left/right select gain (P, I, D, F)
 *   right bumper    selected gain + step
 *   left bumper     selected gain - step
 *   B               intake on / off   EXACTLY TeleOp1: Intake subsystem, toggleWhenPressed(intakeCommand())
 *   X               feeder on / off   EXACTLY TeleOp1: ShooterSubsystem.toggleFeeder()
 *   Dashboard Config: tick/untick intakeOn or feederOn and press Save = same as pressing B / X.
 *   right stick btn step size x10
 *   left stick btn  step size ÷10
 *
 * Why PIDF on a flywheel:
 *   F (feedforward) does most of the work: power needed is roughly proportional to speed,
 *     so F * targetRPM gets the wheel close to the target on its own. It is battery-compensated.
 *   P pushes the speed back up quickly when a ball steals energy from the wheel.
 *   I removes the small steady error that F and P leave behind (keep it small).
 *   D damps overshoot. It is rarely needed on a flywheel (usually 0).
 *
 * Tuning order: P=I=D=0 -> raise F until actual ≈ target -> raise P until it recovers fast
 * after a shot without oscillating -> add a little I only if a steady error remains.
 * On the graph: overshoot = Actual going above Target after spin-up or after a shot.
 */
@TeleOp(name = "Flywheel Tuner", group = "Test")
public class FlywheelTuner extends LinearOpMode {

    // ---- Hardware (change to match your Robot Configuration) ----
    // Same flywheel motor ShooterSubsystem uses (its name lives in ShooterSubsystem.FLYWHEEL_NAME).
    private static final String MOTOR_NAME = ShooterSubsystem.FLYWHEEL_NAME;
    private static final boolean REVERSED = false;
    /**
     * Encoder ticks per motor revolution. The Robot Configuration has the flywheel set to a
     * geared goBILDA type (2786 ticks/rev, 60 RPM), but the flywheel motor is a 1:1 / 6000 RPM
     * goBILDA (28 ticks/rev): measured 2220 ticks/s at full power = ~4760 RPM.
     * Set both to 0 to use the Robot Configuration's motor type instead.
     */
    private static final double TICKS_PER_REV_OVERRIDE = 28;
    private static final double MAX_RPM_OVERRIDE = 6000;

    // ---- Live-tunable from FTC Dashboard (Config tab) ----
    // NOT using @Config on purpose: with FTC SDK 12.0, @Config values saved in the Dashboard
    // don't reach the running code (acmerobotics/ftc-dashboard issue #240). Instead the OpMode
    // registers each value itself at INIT (see registerConfig), so Save writes straight into
    // the variables this loop reads. volatile = safe to change from the Dashboard's thread.
    /** Flywheel on/off. Tick it in the Dashboard Config tab, or press A on gamepad1. */
    public static volatile boolean flywheelOn = false;
    public static volatile double targetRpm = 2000;
    /** Intake / feeder on/off, same behavior as TeleOp1 (B / X). */
    public static volatile boolean intakeOn = false;
    public static volatile boolean feederOn = false;
    public static volatile double kP = 0.0005;
    public static volatile double kI = 0.0;
    public static volatile double kD = 0.0;
    /** 0 = auto: uses 1 / motor max RPM. Set a value above 0 to override. */
    public static volatile double kF = 0.0;

    private static final double RPM_STEP = 20;

    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MAX_INTEGRAL_POWER = 0.2; // anti-windup: I term limited to +/-0.2 power

    private final String[] gainNames = {"P", "I", "D", "F"};
    private final double[] steps = {0.0001, 0.00001, 0.00001, 0.00001};

    private static double getGain(int i) {
        switch (i) { case 0: return kP; case 1: return kI; case 2: return kD; default: return kF; }
    }

    private static void setGain(int i, double v) {
        v = Math.max(0, v);
        switch (i) { case 0: kP = v; break; case 1: kI = v; break; case 2: kD = v; break; default: kF = v; }
    }

    private static final String CATEGORY = "Flywheel Tuner";
    private static final String[] CONFIG_NAMES = {"flywheelOn", "intakeOn", "feederOn", "targetRpm", "kP", "kI", "kD", "kF"};

    private static <T> void addVar(FtcDashboard dashboard, String name, Supplier<T> get, Consumer<T> set) {
        dashboard.addConfigVariable(CATEGORY, name, new ValueProvider<T>() {
            @Override public T get() { return get.get(); }
            @Override public void set(T value) { if (value != null) set.accept(value); }
        });
    }

    /** Put this OpMode's live variables into the Dashboard Config panel. */
    private void registerConfig(FtcDashboard dashboard) {
        addVar(dashboard, "flywheelOn", () -> flywheelOn, v -> flywheelOn = v);
        addVar(dashboard, "intakeOn", () -> intakeOn, v -> intakeOn = v);
        addVar(dashboard, "feederOn", () -> feederOn, v -> feederOn = v);
        addVar(dashboard, "targetRpm", () -> targetRpm, v -> targetRpm = Math.max(0, v));
        addVar(dashboard, "kP", () -> kP, v -> kP = Math.max(0, v));
        addVar(dashboard, "kI", () -> kI, v -> kI = Math.max(0, v));
        addVar(dashboard, "kD", () -> kD, v -> kD = Math.max(0, v));
        addVar(dashboard, "kF", () -> kF, v -> kF = Math.max(0, v));
        dashboard.updateConfig();
    }

    private void unregisterConfig(FtcDashboard dashboard) {
        for (String name : CONFIG_NAMES) {
            try { dashboard.removeConfigVariable(CATEGORY, name); } catch (Exception ignored) { }
        }
    }

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        unregisterConfig(dashboard); // in case a previous run didn't clean up

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        flywheel.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // our own PIDF, encoder still counts
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // coast down, don't brake

        // ---- Intake + feeder: the SAME subsystems and bindings as TeleOp1 ----
        CommandScheduler.getInstance().reset();
        Intake intake = new Intake(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        // ShooterSubsystem puts the flywheel in RUN_USING_ENCODER (hub speed control). The tuner runs its
        // OWN PIDF, so switch it back, otherwise two speed controllers fight each other.
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        GamepadEx driver = new GamepadEx(gamepad1);
        Command intakeToggle = intake.intakeCommand();
        driver.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(intakeToggle);
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(shooter::toggleFeeder, shooter));

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        String motorTypeName = flywheel.getMotorType().getName();
        double ticksPerRev = TICKS_PER_REV_OVERRIDE > 0 ? TICKS_PER_REV_OVERRIDE : flywheel.getMotorType().getTicksPerRev();
        double maxRpm = MAX_RPM_OVERRIDE > 0 ? MAX_RPM_OVERRIDE : flywheel.getMotorType().getMaxRPM();
        if (maxRpm <= 0) maxRpm = 6000;
        double autoKF = 1.0 / maxRpm; // used whenever kF is 0: full power at the motor's free speed

        flywheelOn = false; // always start stopped
        intakeOn = false;
        feederOn = false;
        registerConfig(dashboard);
        boolean wasOn = false;
        int selected = 0;

        boolean lastA = false, lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
        boolean lastRb = false, lastLb = false, lastL3 = false, lastR3 = false;
        boolean lastIntakeOn = false, lastFeederOn = false;

        double integral = 0, lastRpm = 0, voltage = battery.getVoltage();
        long lastNs = 0, lastVoltageNs = 0;

        telemetry.addData("Motor", "%s  (%.1f ticks/rev, max %.0f RPM)", MOTOR_NAME, ticksPerRev, maxRpm);
        telemetry.addLine("Press START (▶), then A to spin up.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            long now = System.nanoTime();
            double dt = lastNs == 0 ? 0 : (now - lastNs) / 1e9;
            lastNs = now;

            // Battery voltage is a slow hub read, so only refresh it 4x per second.
            if (now - lastVoltageNs > 250_000_000L) {
                voltage = battery.getVoltage();
                lastVoltageNs = now;
            }

            // ---------------- Buttons (edge detected) ----------------
            String before = flywheelOn + "|" + intakeOn + "|" + feederOn + "|" + targetRpm + "|" + kP + "|" + kI + "|" + kD + "|" + kF;
            if (gamepad1.a && !lastA) flywheelOn = !flywheelOn;
            boolean on = flywheelOn;
            if (on != wasOn) integral = 0; // fresh start every time it's switched on/off
            wasOn = on;
            if (gamepad1.dpad_up && !lastUp) targetRpm += RPM_STEP;
            if (gamepad1.dpad_down && !lastDown) targetRpm = Math.max(0, targetRpm - RPM_STEP);
            if (gamepad1.dpad_right && !lastRight) selected = (selected + 1) % 4;
            if (gamepad1.dpad_left && !lastLeft) selected = (selected + 3) % 4;
            if (gamepad1.right_bumper && !lastRb) setGain(selected, getGain(selected) + steps[selected]);
            if (gamepad1.left_bumper && !lastLb) setGain(selected, getGain(selected) - steps[selected]);
            if (gamepad1.right_stick_button && !lastR3) steps[selected] *= 10;
            if (gamepad1.left_stick_button && !lastL3) steps[selected] /= 10;

            lastA = gamepad1.a; lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
            lastRb = gamepad1.right_bumper; lastLb = gamepad1.left_bumper;
            lastL3 = gamepad1.left_stick_button; lastR3 = gamepad1.right_stick_button;

            // ---------------- Intake + feeder ----------------
            // 1. Dashboard toggled one? -> do exactly what pressing B / X does.
            if (intakeOn != lastIntakeOn && intakeOn != intakeToggle.isScheduled()) {
                if (intakeOn) intakeToggle.schedule(); else intakeToggle.cancel();
            }
            if (feederOn != lastFeederOn && feederOn != shooter.isFeederOn()) shooter.toggleFeeder();
            // 2. Run TeleOp1's bindings (B / X) and commands.
            CommandScheduler.getInstance().run();
            // 3. The real state is what the subsystems say (covers gamepad presses too).
            intakeOn = intakeToggle.isScheduled();
            feederOn = shooter.isFeederOn();
            lastIntakeOn = intakeOn;
            lastFeederOn = feederOn;

            // Gamepad changed a value -> show it in the Dashboard Config panel too.
            String after = flywheelOn + "|" + intakeOn + "|" + feederOn + "|" + targetRpm + "|" + kP + "|" + kI + "|" + kD + "|" + kF;
            if (!after.equals(before)) dashboard.updateConfig();

            // ---------------- PIDF ----------------
            double rawTicksPerSec = flywheel.getVelocity();
            int encoderPosition = flywheel.getCurrentPosition();
            double rpm = rawTicksPerSec / ticksPerRev * 60.0;
            double error = targetRpm - rpm;
            double power = 0;
            double pTerm = 0, iTerm = 0, dTerm = 0, fTerm = 0;

            if (on) {
                if (dt > 0) {
                    integral += error * dt;
                    if (kI > 0) { // anti-windup: never let I alone exceed MAX_INTEGRAL_POWER
                        double limit = MAX_INTEGRAL_POWER / kI;
                        integral = Math.max(-limit, Math.min(limit, integral));
                    }
                }
                double derivative = dt > 0 ? -(rpm - lastRpm) / dt : 0; // on measurement: no kick on target change

                pTerm = kP * error;
                iTerm = kI * integral;
                dTerm = kD * derivative;
                double f = kF > 0 ? kF : autoKF;
                fTerm = f * targetRpm * (NOMINAL_VOLTAGE / voltage); // battery compensated

                power = Math.max(0.0, Math.min(1.0, pTerm + iTerm + dTerm + fTerm));
            }
            lastRpm = rpm;
            flywheel.setPower(power);


            // ---------------- FTC Dashboard graph (numbers only) ----------------
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target RPM", on ? targetRpm : 0);
            packet.put("Actual RPM", rpm);
            packet.put("Error RPM", on ? error : 0);
            packet.put("Power x1000", power * 1000); // scaled so it fits on the same graph
            packet.put("Feeder ON (500)", feederOn ? 500 : 0); // marks when balls are being fed in
            dashboard.sendTelemetryPacket(packet);

            // ---------------- Driver Station telemetry ----------------
            telemetry.addData("Flywheel", on ? "ON" : "OFF  (press A, or tick flywheelOn in Dashboard Config)");
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("Actual RPM", "%.0f", rpm);
            telemetry.addData("Error RPM", "%.0f", error);
            telemetry.addData("Power", "%.3f", power);
            telemetry.addData("Battery", "%.2f V", voltage);
            telemetry.addData("Intake (B)", intakeOn ? "ON" : "OFF");
            telemetry.addData("Feeder (X)", feederOn ? "ON" : "OFF");
            telemetry.addLine();
            for (int i = 0; i < 4; i++) {
                telemetry.addData((i == selected ? ">> " : "   ") + gainNames[i],
                        "%.6f   (step %.6f)", getGain(i), steps[i]);
            }
            if (kF <= 0) telemetry.addData("F in use", "%.6f (auto, kF = 0)", autoKF);
            telemetry.addData("Terms P / I / D / F", "%.3f / %.3f / %.3f / %.3f", pTerm, iTerm, dTerm, fTerm);
            telemetry.addData("Loop", "%.0f Hz", dt > 0 ? 1.0 / dt : 0);
            telemetry.addLine();
            telemetry.addLine("--- Encoder check ---");
            telemetry.addData("Motor type (config)", motorTypeName);
            telemetry.addData("Ticks per rev used", "%.1f", ticksPerRev);
            telemetry.addData("Raw velocity", "%.0f ticks/s", rawTicksPerSec);
            telemetry.addData("Encoder position", encoderPosition);
            telemetry.update();
        }

        flywheel.setPower(0);
        intakeToggle.cancel();                       // stops the intake (its command's end)
        if (shooter.isFeederOn()) shooter.toggleFeeder(); // stops the feeder
        CommandScheduler.getInstance().reset();
        unregisterConfig(dashboard);
        intakeOn = false;
        feederOn = false;
    }
}
