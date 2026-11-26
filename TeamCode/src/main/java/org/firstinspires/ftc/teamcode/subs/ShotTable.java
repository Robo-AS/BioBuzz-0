package org.firstinspires.ftc.teamcode.subs;

public final class ShotTable {
    private ShotTable() {}

    // distance (in), flywheel target RPM, hood position (0.0 - 0.5)
    public static double[][] TABLE = {
            { 35.41, 3690, 0.50 },
            { 53.29, 3700, 0.50 },
            { 56.78, 3800, 0.50 },
            { 60.15, 3790, 0.50 },
            { 64.22, 3800, 0.47 },
            { 66.28, 3790, 0.47 },
            { 76.91, 3800, 0.44 },
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

    public static double distance(double x, double y, double goalX, double goalY) {
        return Math.hypot(goalX - x, goalY - y);
    }
}