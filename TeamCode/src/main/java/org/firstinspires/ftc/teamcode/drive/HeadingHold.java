package org.firstinspires.ftc.teamcode.drive;

/**
 * Heading hold for free-roam driving (moved out of the TeleOps, logic unchanged).
 * Stick turning -> driver controls rotation. Stick released -> wait SETTLE_NS for the
 * robot to stop spinning, lock that heading, then hold it with a P controller.
 */
public class HeadingHold {
    public static double KP = 0.8;
    public static double MAX_TURN = 0.25;
    public static double STICK_DEADZONE = 0.08;
    public static double TOLERANCE_RAD = Math.toRadians(1.0);
    public static long SETTLE_NS = 200_000_000L;

    private double heldHeading;
    private boolean ready;
    private long settleStartNs;
    private boolean wasManualTurn;
    private double lastError;

    /** Start a fresh settle period (on drive start, after auto aim, after a pose reset). */
    public void reset() {
        ready = false;
        settleStartNs = System.nanoTime();
    }

    /** @return turn power for this loop. */
    public double update(double stickTurn, double heading, MecanumDrive drive) {
        boolean manualTurn = Math.abs(stickTurn) > STICK_DEADZONE;
        long now = System.nanoTime();
        double turnPower;

        if (manualTurn) {
            heldHeading = heading;
            ready = false;
            turnPower = drive.addons(stickTurn);
        } else {
            if (wasManualTurn) {
                settleStartNs = now;
                ready = false;
            }

            if (!ready) {
                heldHeading = heading;
                turnPower = 0.0;
                if (now - settleStartNs >= SETTLE_NS) ready = true;
            } else {
                double error = MecanumDrive.angleWrap(heldHeading - heading);
                turnPower = KP * error * MecanumDrive.AIM_TURN_SIGN;
                turnPower = Math.max(-MAX_TURN, Math.min(MAX_TURN, turnPower));
                if (Math.abs(error) < TOLERANCE_RAD) turnPower = 0.0;
            }
        }

        wasManualTurn = manualTurn;
        lastError = MecanumDrive.angleWrap(heldHeading - heading);
        return turnPower;
    }

    public double heldHeading() { return heldHeading; }
    public double lastError() { return lastError; }
}
