package org.firstinspires.ftc.teamcode.aiming;

import java.util.Arrays;
import org.firstinspires.ftc.teamcode.ShootingConfig.HoodConfig;

/** Conservative travel-time model for a position servo without position feedback. */
public final class HoodController {
    private final HoodConfig c;
    private double target = Double.NaN, motionEnds, readyAfter = Double.POSITIVE_INFINITY;
    private double[] calibration;
    private boolean requested;

    public HoodController(HoodConfig config) { c = config; }

    private double[] calibration() {
        return new double[] {c.minDeg, c.maxDeg, c.positionAtMin, c.positionAtMax,
                c.settleS, c.secondsPerDegree};
    }

    public boolean validConfig() {
        return c.calibrated && AimMath.finite(calibration()) && c.maxDeg > c.minDeg
                && c.positionAtMin >= 0 && c.positionAtMin <= 1
                && c.positionAtMax >= 0 && c.positionAtMax <= 1
                && c.positionAtMin != c.positionAtMax && c.settleS > 0 && c.secondsPerDegree > 0;
    }

    /** Returns servo position, or NaN without requesting movement on invalid input. */
    public double request(double degrees, double now) {
        if (!validConfig() || !AimMath.finite(degrees, now) || degrees < c.minDeg || degrees > c.maxDeg) {
            stop(); return Double.NaN;
        }
        double[] current = calibration();
        if (!Arrays.equals(current, calibration)) stop();
        calibration = current;
        if (!Double.isFinite(target)) {
            // Position is unknown at startup or after cancellation/calibration changes.
            motionEnds = now + (c.maxDeg - c.minDeg) * c.secondsPerDegree;
            target = degrees;
            readyAfter = motionEnds + c.settleS;
        } else if (Math.abs(degrees - target) > 0.05) {
            // Do not discard unfinished travel when a new target arrives mid-move.
            motionEnds = Math.max(now, motionEnds) + Math.abs(degrees - target) * c.secondsPerDegree;
            target = degrees;
            readyAfter = motionEnds + c.settleS;
        }
        requested = true;
        // Small updates inside the deadband do not send a new, unaccounted movement.
        return c.positionAtMin + (target - c.minDeg) / (c.maxDeg - c.minDeg)
                * (c.positionAtMax - c.positionAtMin);
    }

    public boolean atTarget(double now) {
        return requested && validConfig() && Arrays.equals(calibration, calibration())
                && Double.isFinite(now) && now >= readyAfter;
    }
    public double targetDegrees() { return target; }
    public void stop() {
        requested = false; target = Double.NaN; readyAfter = Double.POSITIVE_INFINITY;
    }
}
