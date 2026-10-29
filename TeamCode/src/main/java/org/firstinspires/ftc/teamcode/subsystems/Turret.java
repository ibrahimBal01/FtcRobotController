package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TurretConfig;

public class Turret extends SubsystemBase {

    private final String name;
    private final TurretConfig cfg;
    private final CRServo motor;
    private final DcMotorEx encoder;
    private final VoltageSensor battery;
    private final PIDController pid = new PIDController(0,0,0);
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage;

    private double angleOffsetDegrees = 0;
    private double lastPower = 0;
    private double targetAngle;
    private double targetVelocity = 0; // deg/s the target is moving at (feedforward)



    public Turret(HardwareMap hardwareMap, String name, TurretConfig config) {
        this.name = name;
        this.cfg = config;

        motor = hardwareMap.get(CRServo.class, cfg.getMotorName());
        motor.setDirection(cfg.getReversed() ?
                CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);

        encoder = hardwareMap.get(DcMotorEx.class, cfg.getEncoderName());
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();
    }

    @Override
    public void periodic() {
        double measured = getAngleDegrees();
        double power;

        if (voltageTimer.milliseconds() > 50) {
            double v = battery.getVoltage();
            if (v > 5) voltage = v;
            voltageTimer.reset();
        }

        double error = targetAngle - measured;

        double voltageScale = cfg.useVoltageComp ? cfg.nominalVoltage / voltage : 1.0;

        // Velocity feedforward: kV is motor power per (degree/second) of target motion.
        double feedforward = cfg.kV * targetVelocity * voltageScale;

        if(Math.abs(error) <= cfg.TOLERANCE_DEGREES){
            pid.reset();
            power = feedforward;
        } else {
            pid.setPID(cfg.kP, cfg.kI, cfg.kD);
            power = pid.calculate(measured, targetAngle) + Math.signum(error) * cfg.kS + feedforward;
        }



        if ((measured <= cfg.MIN_DEGREES && power < 0) || (measured >= cfg.MAX_DEGREES && power > 0)) {
            power = 0;
        }

        lastPower = Math.max(-1, Math.min(1, power));
        motor.setPower(lastPower);
    }



    public double getAngleDegrees(){
        double encoderRevs = cfg.ENCODER_SIGN * encoder.getCurrentPosition() / cfg.TICKS_PER_ENCODER_REV;
        return encoderRevs / cfg.ENCODER_REVS_PER_TURRET_REV * 360.0 + angleOffsetDegrees;
    }

    public void setTargetAngle(double degrees) {
        setTargetAngle(degrees, 0.0);
    }

    /**
     * @param degrees            target angle (clamped to MIN..MAX)
     * @param velocityDegPerSec  how fast the target itself is moving (0 for a fixed target);
     *                           used for feedforward so a moving target is tracked without lag
     */
    public void setTargetAngle(double degrees, double velocityDegPerSec) {
        targetAngle = Math.max(cfg.MIN_DEGREES, Math.min(cfg.MAX_DEGREES, degrees));
        targetVelocity = velocityDegPerSec;
    }

    public double getTargetAngle() { return targetAngle; }
    public double getMinDegrees() { return cfg.MIN_DEGREES; }
    public double getMaxDegrees() { return cfg.MAX_DEGREES; }

    /**
     * A direction (headingDeg) can be reached by several turret angles when the turret can turn
     * more than 360 degrees: headingDeg + 360*k. This returns the one that lies inside
     * [minDeg, maxDeg] and is closest to referenceDeg (normally the previous target, so the
     * turret only unwinds when it is forced to by a limit).
     * If the direction is not reachable at all, it returns the closest limit.
     */
    public static double chooseAngle(double headingDeg, double referenceDeg, double minDeg, double maxDeg) {
        int kMin = (int) Math.ceil((minDeg - headingDeg) / 360.0);
        int kMax = (int) Math.floor((maxDeg - headingDeg) / 360.0);

        if (kMin > kMax) { // no multiple of 360 puts this direction inside the limits
            double toMin = Math.abs(wrap180(headingDeg - minDeg));
            double toMax = Math.abs(wrap180(headingDeg - maxDeg));
            return toMin <= toMax ? minDeg : maxDeg;
        }

        double best = headingDeg + 360.0 * kMin;
        for (int k = kMin + 1; k <= kMax; k++) {
            double candidate = headingDeg + 360.0 * k;
            if (Math.abs(candidate - referenceDeg) < Math.abs(best - referenceDeg)) {
                best = candidate;
            }
        }
        return best;
    }

    /** Wraps an angle in degrees into (-180, 180]. */
    public static double wrap180(double deg) {
        double a = deg % 360.0;
        if (a > 180.0) a -= 360.0;
        if (a <= -180.0) a += 360.0;
        return a;
    }

    public boolean atTarget() {
        return Math.abs(targetAngle - getAngleDegrees()) <= cfg.TOLERANCE_DEGREES;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData( name + "Turret angle", getAngleDegrees());
        telemetry.addData(name + "Turret target", targetAngle);
        telemetry.addData(name + "Turret power", lastPower);
        telemetry.addData(name + "Turret On Target", atTarget());
    }

    public void stop(){ motor.setPower(0); }

}