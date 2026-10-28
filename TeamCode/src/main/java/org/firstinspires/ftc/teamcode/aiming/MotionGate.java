package org.firstinspires.ftc.teamcode.aiming;

/** Consume each localization sample once, even when feed and tracking execute in either order. */
public final class MotionGate {
    private double time=Double.NaN, x, y, heading, speed=Double.NaN, turnSpeed=Double.NaN;
    public void reset() { time=Double.NaN; speed=Double.NaN; turnSpeed=Double.NaN; }
    public boolean update(double px,double py,double radians,double sampleTime,double now,
                          double maxSpeed,double maxTurnSpeed) {
        if (!AimMath.finite(px,py,radians,sampleTime,now,maxSpeed,maxTurnSpeed)
                || maxSpeed<=0 || maxTurnSpeed<=0 || now<sampleTime || now-sampleTime>0.15) {
            reset(); return false;
        }
        if (sampleTime==time) {
            // A pose reset without a new localization sample cannot inherit old readiness.
            if (px!=x || py!=y || radians!=heading) { reset(); return false; }
        } else {
            double dt=sampleTime-time;
            if (Double.isFinite(time) && dt>0 && dt<0.25) {
                speed=Math.hypot(px-x,py-y)/dt;
                turnSpeed=Math.abs(AimMath.wrap(Math.toDegrees(radians-heading)))/dt;
            } else { speed=Double.NaN; turnSpeed=Double.NaN; }
            x=px; y=py; heading=radians; time=sampleTime;
        }
        return speed<=maxSpeed && turnSpeed<=maxTurnSpeed;
    }
}
