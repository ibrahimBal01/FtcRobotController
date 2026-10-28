package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ShooterConfig;

/**
 * One flywheel (one or more motors) with closed-loop RPM control. Create one instance per
 * shooter, each with its own ShooterConfig.
 *
 * power = kV * targetRpm * (nominalVoltage / batteryVoltage)   (feedforward, does most of the work)
 *       + PID(rpm error)                                       (corrects load/ball dips)
 * with a full-power recovery boost when far below target.
 *
 * The control loop runs in periodic(), so just call setTargetRpm() / stop() from commands.
 */
public class Shooter extends SubsystemBase {
    private final String name;
    private final ShooterConfig cfg;
    private final DcMotorEx[] motors;
    private final VoltageSensor battery;

    private final PIDController pid = new PIDController(0, 0, 0);
    private final ElapsedTime stableTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private double targetRpm = 0;
    private double filteredRpm = 0;
    private double voltage;
    private double rawRpm = 0;
    private double lastPower = 0;

    /** @param name short label used in telemetry keys, e.g. "Large" */
    public Shooter(HardwareMap hardwareMap, String name, ShooterConfig config) {
        this.name = name;
        this.cfg = config;

        String[] names = cfg.motorNames();
        boolean[] reversed = cfg.reversed();
        motors = new DcMotorEx[names.length];
        for (int i = 0; i < names.length; i++) {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, names[i]);
            m.setDirection(reversed[i]
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // We do our own velocity control; the encoders are only used for reading velocity.
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Let the flywheel coast when power is cut instead of braking hard.
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motors[i] = m;
        }

        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();
    }

    @Override
    public void periodic() {
        double measured = measureRpm();
        rawRpm = measured;
        filteredRpm += cfg.velocityFilterAlpha * (measured - filteredRpm);

        if (targetRpm <= 0) {
            setPower(0);
            stableTimer.reset();
            return;
        }

        // Re-read battery voltage often but smooth it (it sags under load).
        if (voltageTimer.milliseconds() > 50) {
            double v = battery.getVoltage();
            if (v > 5) voltage += cfg.voltageFilterAlpha * (v - voltage);
            voltageTimer.reset();
        }

        double error = targetRpm - filteredRpm;

        if (cfg.useRecoveryBoost && error > cfg.recoveryThresholdRpm) {
            // Far below target (ball dip / spin-up): full authority, keep the integral from winding up.
            pid.reset();
            setPower(cfg.recoveryPower);
        } else {
            pid.setPID(cfg.kP, cfg.kI, cfg.kD);
            pid.setIntegrationBounds(-cfg.integralLimit, cfg.integralLimit);

            double voltageScale = cfg.useVoltageComp ? cfg.nominalVoltage / voltage : 1.0;
            double feedforward = cfg.kV * targetRpm * voltageScale;
            double output = feedforward + pid.calculate(filteredRpm, targetRpm);

            // The flywheel only ever spins one way; no reverse power.
            setPower(Math.max(0, Math.min(1, output)));
        }

        if (Math.abs(targetRpm - filteredRpm) > cfg.rpmTolerance) {
            stableTimer.reset();
        }
    }

    public void setTargetRpm(double rpm) {
        double clamped = Math.max(0, Math.min(cfg.maxRpm(), rpm));
        if (clamped != targetRpm) {
            pid.reset();
            targetRpm = clamped;
        }
    }

    public void stop() {
        setTargetRpm(0);
    }

    /** True once the flywheel has stayed within tolerance of the target for stableMs. */
    public boolean atSpeed() {
        return targetRpm > 0 && stableTimer.milliseconds() >= cfg.stableMs;
    }

    /** True when RPM is within feedToleranceRpm of target right now (no settle time). */
    public boolean readyToFeed() {
        return targetRpm > 0 && Math.abs(targetRpm - filteredRpm) <= cfg.feedToleranceRpm;
    }

    public ShooterConfig getConfig() {
        return cfg;
    }

    public double getRpm() {
        return filteredRpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getRawRpm() {
        return rawRpm;
    }

    public double getPower() {
        return lastPower;
    }

    /**
     * Adds this shooter's tuning values, prefixed with its name. Every value is numeric and the
     * same keys are added on every call, so FTC Dashboard's graph can plot them without flickering.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData(name + " target RPM", targetRpm);
        telemetry.addData(name + " RPM", filteredRpm);
        telemetry.addData(name + " raw RPM", rawRpm);
        telemetry.addData(name + " error RPM", targetRpm - filteredRpm);
        telemetry.addData(name + " power", lastPower);
        telemetry.addData(name + " at speed (0/1)", atSpeed() ? 1 : 0);
    }

    private double measureRpm() {
        // getVelocity() is in ticks/second; average all motors on this flywheel.
        double sum = 0;
        for (DcMotorEx m : motors) sum += Math.abs(m.getVelocity());
        return (sum / motors.length) * 60.0 / cfg.ticksPerRev();
    }

    private void setPower(double power) {
        lastPower = power;
        for (DcMotorEx m : motors) m.setPower(power);
    }
}
