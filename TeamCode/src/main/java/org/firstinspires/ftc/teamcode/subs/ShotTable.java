package org.firstinspires.ftc.teamcode.subs;

/**
 * Distance-to-hive -> flywheel RPM + hood position, with linear interpolation between rows.
 *
 * Fill it with the Shooter Tuner: drive to a spot, find the RPM + hood that score, press A,
 * repeat at several distances, then paste the printed lines below (sorted by distance).
 * Outside the table's range the closest row is used (no extrapolation).
 *
 * !! PLACEHOLDER VALUES — replace with your tuned rows !!
 */
public final class ShotTable {
    private ShotTable() {}

    //                      distance (in), flywheel RPM, hood position (0.0 - 0.5)
    public static double[][] TABLE = {
            { 40, 2000, 0.00 },
            { 80, 2600, 0.20 },
            { 120, 3200, 0.40 },
    };

    public static double rpm(double distance) { return interpolate(distance, 1); }
    public static double hood(double distance) { return interpolate(distance, 2); }

    private static double interpolate(double d, int column) {
        double[][] t = TABLE;
        if (d <= t[0][0]) return t[0][column];
        for (int i = 1; i < t.length; i++) {
            if (d <= t[i][0]) {
                double f = (d - t[i - 1][0]) / (t[i][0] - t[i - 1][0]);
                return t[i - 1][column] + f * (t[i][column] - t[i - 1][column]);
            }
        }
        return t[t.length - 1][column];
    }

    /** Straight-line distance from the robot (x, y) to the hive (goalX, goalY), Pinpoint units (inches). */
    public static double distance(double x, double y, double goalX, double goalY) {
        return Math.hypot(goalX - x, goalY - y);
    }
}
