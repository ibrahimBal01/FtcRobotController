package org.firstinspires.ftc.teamcode.aiming;

/** Interpolates measured data; invalid, uncalibrated and out-of-range requests return null. */
public final class ShotTable {
    private ShotTable() { }
    public static double[] lookup(double[][] rows, double distance) {
        if (!Double.isFinite(distance) || rows == null || rows.length < 2) return null;
        double previous = -1;
        for (double[] row : rows) {
            if (row == null || row.length != 3 || !AimMath.finite(row)
                    || row[0] <= previous || row[1] <= 0) return null;
            previous = row[0];
        }
        if (distance < rows[0][0] || distance > rows[rows.length - 1][0]) return null;
        for (int i = 1; i < rows.length; i++) {
            if (distance <= rows[i][0]) {
                double[] a = rows[i-1], b = rows[i];
                double t = (distance - a[0]) / (b[0] - a[0]);
                return new double[] {a[1] + t * (b[1]-a[1]), a[2] + t * (b[2]-a[2])};
            }
        }
        return null;
    }
}
