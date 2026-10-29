package org.firstinspires.ftc.teamcode.commands.Turret;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Keeps the turret pointed at a fixed point on the field while the robot drives and turns.
 *
 * Every loop:
 *  1. read the robot pose from Pedro (x, y inches, heading radians, counter-clockwise +),
 *  2. compute the field direction from the turret to the point,
 *  3. subtract the robot heading to get the turret angle relative to the robot,
 *  4. pick the equivalent angle inside the turret's limits (Turret.chooseAngle),
 *  5. send it to the turret together with a feedforward velocity (-robot turn rate), so the
 *     turret counter-rotates smoothly instead of lagging behind a spinning robot.
 *
 * Conventions: turret angle 0 = robot forward and positive = counter-clockwise (same as the
 * robot heading). Runs until cancelled (e.g. whileHeld); on end the turret holds where it is.
 * Requires the turret only, so the drive keeps working.
 */
public class HoldPointCommand extends CommandBase {
    /** Low-pass on the robot turn rate (1.0 = none). */
    private static final double OMEGA_FILTER_ALPHA = 0.3;

    private final Drive drive;
    private final Turret turret;
    private final double pointX, pointY;            // field, inches
    private final double offsetForward, offsetLeft; // turret position relative to robot center, inches
    private final double zeroOffsetDeg;             // turret 0 deg relative to robot forward

    private final ElapsedTime timer = new ElapsedTime();
    private double lastHeadingRad;
    private double lastTimeSec;
    private double omegaRadPerSec;
    private double lastChosenAngle;

    public HoldPointCommand(Turret turret, Drive drive, Pose point) {
        this(turret, drive, point.x(), point.y(), 0, 0, 0);
    }

    /**
     * @param offsetForward  how far in front of the robot center the turret axis is (inches)
     * @param offsetLeft     how far left of the robot center the turret axis is (inches)
     * @param zeroOffsetDeg  direction the turret points at angle 0, relative to robot forward
     */
    public HoldPointCommand(Turret turret, Drive drive, double pointX, double pointY,
                            double offsetForward, double offsetLeft, double zeroOffsetDeg) {
        this.turret = turret;
        this.drive = drive;
        this.pointX = pointX;
        this.pointY = pointY;
        this.offsetForward = offsetForward;
        this.offsetLeft = offsetLeft;
        this.zeroOffsetDeg = zeroOffsetDeg;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        timer.reset();
        lastTimeSec = 0;
        lastHeadingRad = drive.getPose().heading();
        omegaRadPerSec = 0;
        lastChosenAngle = turret.getAngleDegrees();
    }

    @Override
    public void execute() {
        Pose pose = drive.getPose();
        double heading = pose.heading();

        // Robot turn rate from the heading change (wrapped so 179 -> -179 degrees is a small step)
        double now = timer.seconds();
        double dt = now - lastTimeSec;
        if (dt > 1e-3) {
            double dHeading = Math.atan2(Math.sin(heading - lastHeadingRad), Math.cos(heading - lastHeadingRad));
            omegaRadPerSec += OMEGA_FILTER_ALPHA * (dHeading / dt - omegaRadPerSec);
            lastHeadingRad = heading;
            lastTimeSec = now;
        }

        // Turret axis position on the field (rotate the robot-frame offset by the heading)
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double turretX = pose.x() + offsetForward * cos - offsetLeft * sin;
        double turretY = pose.y() + offsetForward * sin + offsetLeft * cos;

        // Direction to the point on the field, then relative to the robot
        double fieldAngleDeg = Math.toDegrees(Math.atan2(pointY - turretY, pointX - turretX));
        double desiredDeg = Turret.wrap180(fieldAngleDeg - Math.toDegrees(heading) - zeroOffsetDeg);

        // Equivalent angle inside the limits, staying near the previous choice (no needless unwinding)
        double chosen = Turret.chooseAngle(desiredDeg, lastChosenAngle,
                turret.getMinDegrees(), turret.getMaxDegrees());
        lastChosenAngle = chosen;

        // Robot turns CCW at omega -> turret must turn the opposite way relative to the robot
        turret.setTargetAngle(chosen, -Math.toDegrees(omegaRadPerSec));
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTargetAngle(turret.getAngleDegrees(), 0); // hold here, stop feedforward
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
