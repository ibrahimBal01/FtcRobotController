package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * Hardware description + tuning values for one flywheel shooter.
 * Create one instance per shooter (see Constants.Shooters). The public non-final fields show up
 * in FTC Dashboard's Configuration tab and are re-read every loop, so they can be tuned live.
 * Dashboard edits are live only; copy the final values back into Constants.java.
 */

public class ShooterConfig {
    // ---- Hardware description (fixed in code) ----
    private final String[] motorNames;
    private final boolean[] reversed;
    private final double ticksPerRev;
    private final double maxRpm;
    private final String hoodServoName;
    private final boolean hoodReversed;

    // ---- Tunables ----
    /** Hood servo position (0..1) applied at start-up and by the tuner. */
    public double hoodPosition = 0.5;
    /** Safe travel limits for the hood servo (0..1). Set these so the hood never binds. */
    public double hoodMinPosition = 0.0;
    public double hoodMaxPosition = 1.0;

    /** Target RPM used by the gamepad bindings / tuner. */
    public double shootRpm;


    /** Feedforward: motor power per RPM of target. 1/maxRpm is the theoretical start value. */
    public double kV;
    /** Feedback on RPM error (power per RPM of error). */
    public double kP = 0.0003;
    public double kI = 0.0;
    public double kD = 0.0;
    /** Clamps the accumulated integral error (rpm*s). */
    public double integralLimit = 300.0;

    /** Battery voltage the feedforward was tuned at (scaled by nominal / actual). */
    public double nominalVoltage = 12.0;
    public boolean useVoltageComp = true;
    /** Battery voltage low-pass (1.0 = no filtering). */
    public double voltageFilterAlpha = 0.1;

    /** "At speed" = within rpmTolerance of target continuously for stableMs. */
    public double rpmTolerance = 50.0;
    public double stableMs = 100.0;

    /** Below target by more than recoveryThresholdRpm -> recoveryPower instead of PIDF. */
    public boolean useRecoveryBoost = true;
    public double recoveryThresholdRpm = 100.0;
    public double recoveryPower = 1.0;

    /** Feeder gate: feed when within this many RPM of target (keep above RPM noise). */
    public double feedToleranceRpm = 100.0;

    /** Velocity low-pass filter: 1.0 = no filtering, smaller = smoother but laggier. */
    public double velocityFilterAlpha = 0.4;

    /**
     * @param motorNames  hardware names of the motors driving this flywheel (1 or more)
     * @param reversed    one flag per motor; true = reverse that motor
     * @param ticksPerRev encoder ticks per flywheel revolution (28 for a goBILDA 6000 rpm motor 1:1)
     * @param maxRpm      free speed at the flywheel (6000 for a goBILDA 6000 rpm motor 1:1)
     * @param shootRpm    default target RPM
     * @param hoodServoName hardware name of the hood's position servo
     * @param hoodReversed  true to reverse the hood servo direction
     */
    public ShooterConfig(String[] motorNames, boolean[] reversed, double ticksPerRev,
                         double maxRpm, double shootRpm, String hoodServoName, boolean hoodReversed) {
        if (motorNames.length != reversed.length) {
            throw new IllegalArgumentException("motorNames and reversed must have the same length");
        }
        this.motorNames = motorNames;
        this.reversed = reversed;
        this.ticksPerRev = ticksPerRev;
        this.maxRpm = maxRpm;
        this.hoodServoName = hoodServoName;
        this.hoodReversed = hoodReversed;
        this.shootRpm = shootRpm;
        this.kV = 1.0 / maxRpm;
    }

    public String[] motorNames() { return motorNames; }
    public boolean[] reversed() { return reversed; }
    public double ticksPerRev() { return ticksPerRev; }
    public double maxRpm() { return maxRpm; }
    public String hoodServoName() { return hoodServoName; }
    public boolean hoodReversed() { return hoodReversed; }
}
