package org.firstinspires.ftc.teamcode.aiming;

public final class AimMath {
    private AimMath() { }
    public static double wrap(double degrees) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(degrees)),
                Math.cos(Math.toRadians(degrees))));
    }
    public static double reachableAngle(double desired, double current, double min, double max) {
        if (!finite(desired, current, min, max) || min > max) return Double.NaN;
        double base = wrap(desired);
        double lo = Math.ceil((min - base) / 360), hi = Math.floor((max - base) / 360);
        if (lo > hi) return Double.NaN;
        // No integer loop: large but finite configuration values cannot overflow a loop counter.
        double turns = clamp(Math.rint((current - base) / 360), lo, hi);
        double result = base + 360 * turns;
        return result >= min && result <= max ? result : Double.NaN;
    }
    public static double[] solve(double x, double y, double headingRad,
                                  double forward, double left, double yawOffset,
                                  double targetX, double targetY, double muzzleForward) {
        double px = x + forward * Math.cos(headingRad) - left * Math.sin(headingRad);
        double py = y + forward * Math.sin(headingRad) + left * Math.cos(headingRad);
        double bearing = Math.atan2(targetY - py, targetX - px);
        return new double[] {wrap(Math.toDegrees(bearing - headingRad) - yawOffset),
                Math.hypot(targetX - px, targetY - py) - muzzleForward, px, py};
    }
    public static boolean finite(double... values) {
        for (double v : values) if (!Double.isFinite(v)) return false;
        return true;
    }
    public static double clamp(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }
}
