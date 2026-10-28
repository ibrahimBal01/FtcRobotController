package org.firstinspires.ftc.teamcode.aiming;

import org.firstinspires.ftc.teamcode.ShootingConfig.TurretConfig;

/** Hardware-independent state machine. update() time is monotonic seconds. */
public class TurretController {
    public enum State { UNHOMED, CLEAR_SWITCH, SEEK_HOME, HOLD, FAULT }
    private final TurretConfig c;
    private State state = State.UNHOMED;
    private double zeroTicks, angle, velocity, target, lastTime = Double.NaN, lastAngle;
    private double homeStart, switchSince = Double.NaN, motionSince, motionAngle, lastPower;
    private boolean requested;
    private String fault = "";
    private double[] referenceCalibration;
    public TurretController(TurretConfig config) { c = config; }
    public boolean validConfig() {
        return c.calibrated && AimMath.finite(c.minDeg,c.maxDeg,c.encoderTurnsPerTurretTurn,
                c.ticksPerEncoderTurn,c.maxPower,c.homePower,c.homeTimeoutS,c.debounceS,
                c.kP,c.kD,c.toleranceDeg,c.stallTimeoutS,c.minimumMotionDeg,c.maxMeasuredVelocityDegS,
                c.maxReadyVelocityDegS)
                && c.minDeg < c.maxDeg && c.maxDeg-c.minDeg <= 720
                && c.encoderTurnsPerTurretTurn > 0 && c.ticksPerEncoderTurn > 0
                && Math.abs(c.encoderSign)==1 && Math.abs(c.servoSign)==1
                && c.maxPower > 0 && c.maxPower <= 1 && c.homePower > 0 && c.homePower <= c.maxPower
                && c.homeTimeoutS > 0 && c.debounceS > 0 && c.stallTimeoutS > 0
                && c.kP > 0 && c.kD >= 0 && c.toleranceDeg > 0 && c.minimumMotionDeg > 0
                && c.maxReadyVelocityDegS > 0 && c.maxMeasuredVelocityDegS > 0;
    }
    private double[] referenceCalibration() {
        return new double[] {c.minDeg,c.maxDeg,c.encoderTurnsPerTurretTurn,c.ticksPerEncoderTurn,
                c.encoderSign,c.servoSign,c.limitActiveLow ? 1 : 0};
    }
    private boolean referenceUnchanged() {
        return java.util.Arrays.equals(referenceCalibration, referenceCalibration());
    }
    public void home(double now, boolean switchActive) {
        stop();
        if (!validConfig()) { fail("Turret calibration/configuration required"); return; }
        state = switchActive ? State.CLEAR_SWITCH : State.SEEK_HOME;
        referenceCalibration = referenceCalibration();
        lastTime = Double.NaN;
        homeStart = now; motionSince = now; motionAngle = angle; switchSince = Double.NaN; fault = "";
    }
    public boolean aim(double desired) {
        if (state != State.HOLD || !validConfig() || !referenceUnchanged()) { requested = false; return false; }
        double result = AimMath.reachableAngle(desired, angle, c.minDeg, c.maxDeg);
        requested = Double.isFinite(result);
        if (requested) target = result;
        return requested;
    }
    public double update(double ticks, boolean limit, double now) {
        if (!AimMath.finite(ticks,now)) { fail("Invalid encoder/time"); return 0; }
        double scale = 360.0 * c.encoderSign / (c.ticksPerEncoderTurn * c.encoderTurnsPerTurretTurn);
        angle = (ticks - zeroTicks) * scale + c.minDeg;
        double dt = now - lastTime;
        velocity = Double.isFinite(lastTime) && dt > 0 ? (angle-lastAngle)/dt : 0;
        lastTime = now; lastAngle = angle;
        if (state == State.UNHOMED || state == State.FAULT) return lastPower = 0;
        if (!validConfig() || !AimMath.finite(angle,velocity)) { fail("Invalid configuration/measurement"); return 0; }
        if (!referenceUnchanged()) { fail("Reference calibration changed; re-home"); return 0; }
        if (dt > 0.25) { fail("Control loop stalled; re-home"); return 0; }
        if (Math.abs(velocity) > c.maxMeasuredVelocityDegS) { fail("Encoder discontinuity; re-home"); return 0; }
        double power = 0;
        if (state == State.CLEAR_SWITCH || state == State.SEEK_HOME) {
            if (now-homeStart > c.homeTimeoutS) { fail("Home timeout / stuck switch"); return 0; }
            boolean condition = state == State.CLEAR_SWITCH ? !limit : limit;
            if (!condition) switchSince = Double.NaN;
            else if (!Double.isFinite(switchSince)) switchSince = now;
            if (condition && now-switchSince >= c.debounceS) {
                if (state == State.CLEAR_SWITCH) { state = State.SEEK_HOME; switchSince = Double.NaN; }
                else {
                    zeroTicks = ticks; angle = c.minDeg; lastAngle = angle; velocity = 0;
                    target = angle; requested = false; state = State.HOLD; return lastPower = 0;
                }
            }
            // Stop on first switch assertion while debouncing; never push through the endpoint.
            power = state == State.CLEAR_SWITCH ? c.homePower : (limit ? 0 : -c.homePower);
        } else {
            if (angle < c.minDeg-3 || angle > c.maxDeg+3) { fail("Beyond soft travel limits"); return 0; }
            if (limit && angle > c.minDeg + 5) { fail("Unexpected home switch; re-home"); return 0; }
            if (requested) {
                double error = target-angle;
                power = Math.abs(error) <= c.toleranceDeg ? 0
                        : AimMath.clamp(c.kP*error-c.kD*velocity,-c.maxPower,c.maxPower);
            }
            if ((limit || angle <= c.minDeg) && power < 0 || angle >= c.maxDeg && power > 0) power = 0;
        }
        if (Math.abs(power) < 0.05 || Math.abs(angle-motionAngle) >= c.minimumMotionDeg) {
            motionSince=now; motionAngle=angle;
        } else if (now-motionSince > c.stallTimeoutS) { fail("No encoder movement under power"); return 0; }
        return lastPower = power;
    }
    public void stop() {
        requested = false; lastPower = 0;
        if (state == State.CLEAR_SWITCH || state == State.SEEK_HOME) state = State.UNHOMED;
    }
    private void fail(String reason) { state=State.FAULT; requested=false; lastPower=0; fault=reason; }
    public boolean atTarget() { return state==State.HOLD && requested && validConfig() && referenceUnchanged()
            && Math.abs(target-angle)<=c.toleranceDeg && Math.abs(velocity)<=c.maxReadyVelocityDegS; }
    public State state() { return state; }
    public double angle() { return angle; }
    public double target() { return target; }
    public String fault() { return fault; }
}
