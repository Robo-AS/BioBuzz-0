package org.firstinspires.ftc.teamcode.teleops.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subs.FlywheelPIDF;
import org.firstinspires.ftc.teamcode.subs.HoodServo;
import org.firstinspires.ftc.teamcode.subs.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subs.ShotTable;
import org.firstinspires.ftc.teamcode.teleops.BaseTeleOp;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/*
 * SHOOTER TUNER — build the distance -> (flywheel RPM, hood) table, tune the flywheel PIDF.
 * Full TeleOp (BLUE start + goal, same as TeleOp1): drive, X auto aim, D-pad L/R goal trim, R1 faras+intake.
 *
 * Distance to the hive = sqrt((goalX - x)^2 + (goalY - y)^2) from the Pinpoint pose (inches).
 *
 * FTC Dashboard (192.168.43.1:8080/dash) -> Config -> "Shooter Tuner" (change + Save, live):
 *   flywheelOn, useTable, targetRpm, hood, kP, kI, kD, kF
 *   Graph: "Target RPM", "Actual RPM", "Distance".
 *
 * gamepad1 (on top of the BaseTeleOp buttons):
 *   Y              flywheel on / off
 *   LB             feeder on / off  (shoot)
 *   B              intake on / off
 *   dpad up/down   target RPM +50 / -50       (manual mode)
 *   L3 / R3        hood -0.02 / +0.02         (stick clicks, manual mode)
 *   back / share   switch MANUAL <-> TABLE    (TABLE = RPM + hood from ShotTable by distance)
 *   A              SAVE this shot: {distance, RPM, hood} -> printed as Java lines in telemetry.
 *                  Paste the lines into ShotTable.TABLE (they are already sorted by distance).
 *
 * Workflow: put robot on the TeleOp1 start spot -> INIT/START -> drive to a distance -> X (aim) ->
 * Y (spin up) -> adjust RPM + hood until shots score -> A (save) -> next distance.
 */
@TeleOp(name = "Shooter Tuner", group = "Test")
public class ShooterTuner extends BaseTeleOp {

    // BLUE, same as TeleOp1
    private static final PoseFactory P = PoseFactory.degrees();
    private static final Pose START = P.of(132, 9, 90);
    private static final double GOAL_X = 77;
    private static final double GOAL_Y = 71;

    @Override protected Pose startPose() { return START; }
    @Override protected double goalX() { return GOAL_X; }
    @Override protected double goalY() { return GOAL_Y; }

    // ---- Live values (Dashboard + gamepad). volatile: changed from the Dashboard thread ----
    private static volatile boolean flywheelOn = false;
    private static volatile boolean useTable = false;
    private static volatile double targetRpm = 2000;
    private static volatile double hoodPos = 0.0;

    private ShooterSubsystem shooter;
    private HoodServo hood;
    private DcMotorEx flywheel;
    private VoltageSensor battery;
    private final FlywheelPIDF pidf = new FlywheelPIDF();

    private double distance = 0, actualRpm = 0, power = 0, voltage = 12;
    private long lastVoltageNs = 0;
    private final List<double[]> saved = new ArrayList<>();

    @Override
    protected void configureBindings() {
        shooter = new ShooterSubsystem(hardwareMap);   // feeder (and it grabs the flywheel motor)
        hood = new HoodServo(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, ShooterSubsystem.FLYWHEEL_NAME);
        // ShooterSubsystem sets RUN_USING_ENCODER (hub speed control). We run our own PIDF -> switch back.
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();

        flywheelOn = false;
        useTable = false;
        hoodPos = hood.getPosition();

        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> { flywheelOn = !flywheelOn; pidf.reset(); }));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(shooter::toggleFeeder, shooter));
        driver.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(intake.intakeCommand());
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> targetRpm += 50));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> targetRpm = Math.max(0, targetRpm - 50)));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> hoodPos = Math.max(0, hoodPos - 0.02)));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> hoodPos = Math.min(0.5, hoodPos + 0.02)));
        driver.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> useTable = !useTable));
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(this::saveShot));

        registerDashboardConfig();
    }

    @Override
    protected void onStart() {
        super.onStart();
        schedule(new RunCommand(this::updateShooter)); // runs every loop
    }

    private void updateShooter() {
        Pose pose = drive.pose();
        distance = ShotTable.distance(pose.x(), pose.y(),
                drive.mecanum().getGoalX(), drive.mecanum().getGoalY());

        if (useTable) {
            targetRpm = ShotTable.rpm(distance);
            hoodPos = ShotTable.hood(distance);
        }
        if (Math.abs(hood.getPosition() - hoodPos) > 1e-4) hood.setPosition(hoodPos);

        long now = System.nanoTime();
        if (now - lastVoltageNs > 250_000_000L) { voltage = battery.getVoltage(); lastVoltageNs = now; }

        actualRpm = FlywheelPIDF.ticksPerSecToRpm(flywheel.getVelocity());
        power = pidf.update(flywheelOn, targetRpm, actualRpm, voltage);
        flywheel.setPower(power);
    }

    private void saveShot() {
        double[] row = { Math.round(distance * 10) / 10.0, Math.round(targetRpm), Math.round(hoodPos * 1000) / 1000.0 };
        saved.add(row);
        saved.sort((a, b) -> Double.compare(a[0], b[0]));
    }

    @Override
    protected void addTelemetry() {
        super.addTelemetry();
        telemetry.addLine("--- SHOOTER TUNER ---");
        telemetry.addData("Mode", useTable ? "TABLE (from distance)" : "MANUAL");
        telemetry.addData("Distance to hive", "%.1f in", distance);
        telemetry.addData("Flywheel", "%s  target %.0f  actual %.0f RPM  power %.2f",
                flywheelOn ? "ON" : "OFF", targetRpm, actualRpm, power);
        telemetry.addData("Hood", "%.3f", hoodPos);
        telemetry.addData("PIDF", "P %.5f  I %.5f  D %.5f  F %.6f",
                FlywheelPIDF.kP, FlywheelPIDF.kI, FlywheelPIDF.kD, FlywheelPIDF.kF);
        telemetry.addData("Battery", "%.2f V", voltage);
        telemetry.addLine("Saved shots (paste into ShotTable.TABLE):");
        for (double[] r : saved) telemetry.addLine(String.format("  { %.1f, %.0f, %.3f },", r[0], r[1], r[2]));

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target RPM", flywheelOn ? targetRpm : 0);
        packet.put("Actual RPM", actualRpm);
        packet.put("Distance", distance);
        packet.put("Hood x1000", hoodPos * 1000);
        packet.addLine("Saved shots (paste into ShotTable.TABLE):");
        for (double[] r : saved) packet.addLine(String.format("{ %.1f, %.0f, %.3f },", r[0], r[1], r[2]));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    // ---------------- Dashboard live config (works on SDK 12, unlike @Config) ----------------
    private static final String CATEGORY = "Shooter Tuner";

    private static <T> void addVar(FtcDashboard dash, String name, Supplier<T> get, Consumer<T> set) {
        try { dash.removeConfigVariable(CATEGORY, name); } catch (Exception ignored) { }
        dash.addConfigVariable(CATEGORY, name, new ValueProvider<T>() {
            @Override public T get() { return get.get(); }
            @Override public void set(T value) { if (value != null) set.accept(value); }
        });
    }

    private void registerDashboardConfig() {
        FtcDashboard dash = FtcDashboard.getInstance();
        addVar(dash, "flywheelOn", () -> flywheelOn, v -> { flywheelOn = v; pidf.reset(); });
        addVar(dash, "useTable", () -> useTable, v -> useTable = v);
        addVar(dash, "targetRpm", () -> targetRpm, v -> targetRpm = Math.max(0, v));
        addVar(dash, "hood", () -> hoodPos, v -> hoodPos = Math.max(0, Math.min(0.5, v)));
        addVar(dash, "kP", () -> FlywheelPIDF.kP, v -> FlywheelPIDF.kP = Math.max(0, v));
        addVar(dash, "kI", () -> FlywheelPIDF.kI, v -> FlywheelPIDF.kI = Math.max(0, v));
        addVar(dash, "kD", () -> FlywheelPIDF.kD, v -> FlywheelPIDF.kD = Math.max(0, v));
        addVar(dash, "kF", () -> FlywheelPIDF.kF, v -> FlywheelPIDF.kF = Math.max(0, v));
        dash.updateConfig();
    }
}
