package org.firstinspires.ftc.teamcode.subs;

/**
 * Distance to the hive -> flywheel RPM + hood, with linear interpolation between rows.
 *
 * TWO tables, one for the FRONT and one for the BACK of the hive, because shots from the two
 * sides don't match at the same distance. The side is picked automatically from the robot's Y:
 *   robot Y <  goal Y  -> TABLE_BACK   (the side you start on, e.g. BLUE start y = 9)
 *   robot Y >= goal Y  -> TABLE_FRONT  (the other side of the hive)
 * Shared by BLUE and RED: the field halves are identical and distance is measured to your own
 * alliance's goal, so one set of tables works for both colors.
 * (Telemetry shows which side is being used.)
 *
 * Both tables start with the BLUE table that works on one side. Re-tune the side that misses
 * with the Shooter Tuner: drive to that side, find RPM + hood that score, press A. The tuner
 * prints each saved row under the right table name, so you paste it into that table.
 * Outside a table's range the closest row is used (no extrapolation).
 */
public final class ShotTable {
    private ShotTable() {}

    //                          distance (in), flywheel RPM, hood (0.0 - 0.5)
    public static double[][] TABLE_BACK = {
            { 35.41, 3700, 0.50 },
            { 53.29, 3710, 0.50 },
            { 56.78, 3810, 0.50 },
            { 60.15, 3800, 0.50 },
            { 64.22, 3810, 0.45 },
            { 66.28, 3800, 0.45 },
            { 76.91, 3810, 0.40 },
    };

    // Tuned values for the FRONT side
    public static double[][] TABLE_FRONT = {
            { 59.52, 3830, 0.48 },
            { 60.44, 3710, 0.48 },
            { 61.39, 3830, 0.48 },
            { 64.84, 3810, 0.47 },
            { 65.42, 3830, 0.50 },
            { 72.33, 3800, 0.50 },
            { 83.68, 3800, 0.50 },
            { 88.60, 3910, 0.45 }
    };

    /** Which side of the hive the robot is on. */
    public static boolean isFront(double robotY, double goalY) {
        return robotY >= goalY;
    }

    public static String sideName(double robotY, double goalY) {
        return isFront(robotY, goalY) ? "FRONT" : "BACK";
    }

    public static double distance(double x, double y, double goalX, double goalY) {
        return Math.hypot(goalX - x, goalY - y);
    }

    /** RPM for a robot at (x, y): picks the side's table, interpolates by distance. */
    public static double rpm(double x, double y, double goalX, double goalY) {
        return interpolate(table(y, goalY), distance(x, y, goalX, goalY), 1);
    }

    /** Hood position for a robot at (x, y): picks the side's table, interpolates by distance. */
    public static double hood(double x, double y, double goalX, double goalY) {
        return interpolate(table(y, goalY), distance(x, y, goalX, goalY), 2);
    }

    private static double[][] table(double robotY, double goalY) {
        return isFront(robotY, goalY) ? TABLE_FRONT : TABLE_BACK;
    }

    private static double interpolate(double[][] t, double d, int column) {
        if (d <= t[0][0]) return t[0][column];
        for (int i = 1; i < t.length; i++) {
            if (d <= t[i][0]) {
                double f = (d - t[i - 1][0]) / (t[i][0] - t[i - 1][0]);
                return t[i - 1][column] + f * (t[i][column] - t[i - 1][column]);
            }
        }
        return t[t.length - 1][column];
    }
}