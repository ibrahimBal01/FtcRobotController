package org.firstinspires.ftc.teamcode.aiming;

import org.firstinspires.ftc.teamcode.ShootingConfig;

/** The driver selects the currently RAISED end; never selects the opponent's hive. */
public class FieldTargets {
    public enum Alliance { RED, BLUE }
    public enum End { AUDIENCE, SCORING }
    private Alliance alliance = Alliance.RED;
    private End end = End.AUDIENCE;
    public void select(Alliance alliance, End end) { this.alliance = alliance; this.end = end; }
    public Alliance getAlliance() { return alliance; }
    public End getEnd() { return end; }
    public double[] point() {
        double[] p = alliance == Alliance.RED
                ? (end == End.AUDIENCE ? ShootingConfig.RED_AUDIENCE : ShootingConfig.RED_SCORING)
                : (end == End.AUDIENCE ? ShootingConfig.BLUE_AUDIENCE : ShootingConfig.BLUE_SCORING);
        return p == null ? null : p.clone();
    }
    public boolean onEntranceSide(double pivotX, double pivotY) {
        double[] p = point();
        if (p == null || p.length != 3 || !AimMath.finite(p)) return false;
        // In the documented CAD-aligned field frame Audience is -Y, Scoring is +Y.
        return end == End.AUDIENCE ? pivotY < p[1] - 6 : pivotY > p[1] + 6;
    }
}
