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
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Two-motor flywheel with closed-loop RPM control.
 *
 * power = kV * targetRpm * (nominalVoltage / batteryVoltage)   (feedforward, does most of the work)
 *       + PID(rpm error)                                       (corrects load/ball dips)
 *
 * The control loop runs in periodic(), so just call setTargetRpm() / stop() from commands.
 */
public class Shooter extends SubsystemBase {
    private final DcMotorEx left;
    private final DcMotorEx right;
    private final VoltageSensor battery;

    private final PIDController pid = new PIDController(0, 0, 0);
    private final ElapsedTime stableTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private double targetRpm = 0;
    private double filteredRpm = 0;
    private double voltage;
    private double rawRpm = 0;
    private double lastPower = 0;

    public Shooter(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, Constants.Shooter.LEFT_MOTOR);
        right = hardwareMap.get(DcMotorEx.class, Constants.Shooter.RIGHT_MOTOR);

        left.setDirection(Constants.Shooter.LEFT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setDirection(Constants.Shooter.RIGHT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // We do our own velocity control; the encoders are only used for reading velocity.
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Let the flywheel coast when power is cut instead of braking hard.
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();
    }

    @Override
    public void periodic() {
        double measured = measureRpm();
        rawRpm = measured;
        filteredRpm += Constants.Shooter.VELOCITY_FILTER_ALPHA * (measured - filteredRpm);

        if (targetRpm <= 0) {
            setPower(0);
            stableTimer.reset();
            return;
        }

        // Re-read battery voltage a few times a second (it changes slowly).
        if (voltageTimer.milliseconds() > 50) {
            double v = battery.getVoltage();
            if (v > 5) voltage += Constants.Shooter.VOLTAGE_FILTER_ALPHA * (v - voltage);
            voltageTimer.reset();
        }

        double error = targetRpm - filteredRpm;

        if (Constants.Shooter.USE_RECOVERY_BOOST && error > Constants.Shooter.RECOVERY_THRESHOLD_RPM) {
            // Far below target (ball dip / spin-up): full authority, keep the integral from winding up.
            pid.reset();
            setPower(Constants.Shooter.RECOVERY_POWER);
        } else {
            pid.setPID(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD);
            pid.setIntegrationBounds(-Constants.Shooter.INTEGRAL_LIMIT, Constants.Shooter.INTEGRAL_LIMIT);

            double voltageScale = Constants.Shooter.USE_VOLTAGE_COMP
                    ? Constants.Shooter.NOMINAL_VOLTAGE / voltage : 1.0;
            double feedforward = Constants.Shooter.kV * targetRpm * voltageScale;
            double output = feedforward + pid.calculate(filteredRpm, targetRpm);

            // The flywheel only ever spins one way; no reverse power.
            setPower(Math.max(0, Math.min(1, output)));
        }

        if (Math.abs(targetRpm - filteredRpm) > Constants.Shooter.RPM_TOLERANCE) {
            stableTimer.reset();
        }
    }

    public void setTargetRpm(double rpm) {
        double clamped = Math.max(0, Math.min(Constants.Shooter.MAX_RPM, rpm));
        if (clamped != targetRpm) {
            pid.reset();
            targetRpm = clamped;
        }
    }

    public void stop() {
        setTargetRpm(0);
    }

    /** True once the flywheel has stayed within tolerance of the target for STABLE_MS. */
    public boolean atSpeed() {
        return targetRpm > 0 && stableTimer.milliseconds() >= Constants.Shooter.STABLE_MS;
    }

    /** True when RPM is within FEED_TOLERANCE_RPM of target right now (no settle time). */
    public boolean readyToFeed() {
        return targetRpm > 0
                && Math.abs(targetRpm - filteredRpm) <= Constants.Shooter.FEED_TOLERANCE_RPM;
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
     * Adds the flywheel values used for tuning. Every value is numeric and the same keys are
     * added on every call, so FTC Dashboard's graph can plot them without flickering.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter target RPM", targetRpm);
        telemetry.addData("Shooter RPM", filteredRpm);
        telemetry.addData("Shooter raw RPM", rawRpm);
        telemetry.addData("Shooter error RPM", targetRpm - filteredRpm);
        telemetry.addData("Shooter power", lastPower);
        telemetry.addData("Shooter at speed (0/1)", atSpeed() ? 1 : 0);
    }

    private double measureRpm() {
        // getVelocity() is in ticks/second; average both motors.
        double ticksPerSecond = (Math.abs(left.getVelocity()) + Math.abs(right.getVelocity())) / 2.0;
        return ticksPerSecond * 60.0 / Constants.Shooter.TICKS_PER_REV;
    }

    private void setPower(double power) {
        lastPower = power;
        left.setPower(power);
        right.setPower(power);
    }
}
