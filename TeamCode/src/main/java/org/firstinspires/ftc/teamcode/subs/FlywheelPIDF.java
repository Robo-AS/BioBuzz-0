package org.firstinspires.ftc.teamcode.subs;

/**
 * Flywheel speed controller (no hardware inside): target RPM + measured RPM -> motor power.
 * Same math as the Flywheel Tuner. Gains are the values tuned on the Dashboard
 * (kF 0.00021 / kP 0.0005 at 2000 RPM). Re-tune with the Shooter Tuner and copy the results here.
 *
 *   F  = kF * target * (12 V / battery)   -> does most of the work, battery compensated
 *   P  = kP * error                       -> recovers speed after a shot
 *   I  = kI * integral(error)             -> removes a small steady gap (limited, anti-windup)
 *   D  = kD * -d(measured)/dt             -> damping (usually 0)
 */
public class FlywheelPIDF {

    public static volatile double kP = 0.0005;
    public static volatile double kI = 0.0;
    public static volatile double kD = 0.0;
    public static volatile double kF = 0.00021;

    public static final double TICKS_PER_REV = 28;   // goBILDA 6000 RPM (1:1)
    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double MAX_INTEGRAL_POWER = 0.2;

    private double integral = 0;
    private double lastRpm = 0;
    private long lastNs = 0;

    /** Encoder ticks/s (DcMotorEx.getVelocity()) -> RPM. */
    public static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec / TICKS_PER_REV * 60.0;
    }

    public void reset() {
        integral = 0;
        lastNs = 0;
    }

    /** @return motor power 0..1 (0 when off). */
    public double update(boolean on, double targetRpm, double measuredRpm, double batteryVoltage) {
        long now = System.nanoTime();
        double dt = lastNs == 0 ? 0 : (now - lastNs) / 1e9;
        lastNs = now;

        if (!on || targetRpm <= 0) {
            integral = 0;
            lastRpm = measuredRpm;
            return 0;
        }

        double error = targetRpm - measuredRpm;
        if (dt > 0) {
            integral += error * dt;
            if (kI > 0) {
                double limit = MAX_INTEGRAL_POWER / kI;
                integral = Math.max(-limit, Math.min(limit, integral));
            }
        }
        double derivative = dt > 0 ? -(measuredRpm - lastRpm) / dt : 0;
        lastRpm = measuredRpm;

        double voltage = batteryVoltage > 1 ? batteryVoltage : NOMINAL_VOLTAGE;
        double power = kP * error + kI * integral + kD * derivative
                + kF * targetRpm * (NOMINAL_VOLTAGE / voltage);
        return Math.max(0.0, Math.min(1.0, power));
    }
}
