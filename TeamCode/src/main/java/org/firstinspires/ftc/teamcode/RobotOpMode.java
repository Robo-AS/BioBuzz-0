package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

/**
 * Base for every OpMode on the robot (TeleOp, Auto and test tools).
 *
 * Loop order every cycle:
 *   1. clear the hub bulk cache (one read per hub per loop instead of one per call)
 *   2. onStart() on the very first loop
 *   3. CommandScheduler.run()  -> subsystem periodic() -> triggers -> commands
 *   4. telemetry
 */
public abstract class RobotOpMode extends CommandOpMode {

    private List<LynxModule> hubs;
    private boolean started = false;
    private long lastLoopNs = 0;
    protected double loopHz = 0;

    /** Build subsystems, default commands and bindings here. Runs during INIT. */
    protected abstract void initRobot();

    /** Runs once on the first loop after START (schedule auto routines here). */
    protected void onStart() {}

    /** Add telemetry lines. telemetry.update() is called for you. */
    protected void addTelemetry() {}

    @Override
    public final void initialize() {
        // Clear anything left behind by a previous OpMode (the scheduler is a singleton).
        CommandScheduler.getInstance().reset();

        initRobot();

        // Set AFTER the hardware is built so nothing overrides it.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void run() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        if (!started) {
            started = true;
            onStart();
        }

        super.run(); // CommandScheduler.getInstance().run()

        long now = System.nanoTime();
        if (lastLoopNs != 0) loopHz = 1e9 / (now - lastLoopNs);
        lastLoopNs = now;

        telemetry.addData("Loop (Hz)", "%.0f", loopHz);
        addTelemetry();
        telemetry.update();
    }
}
