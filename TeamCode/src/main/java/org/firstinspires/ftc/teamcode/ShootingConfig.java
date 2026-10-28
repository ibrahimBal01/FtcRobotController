package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/** All lengths INCHES, angles DEGREES here. Pedro heading is radians. */
@Config
public final class ShootingConfig {
    private ShootingConfig() { }
    public static Channel LARGE = new Channel("large", 225.0 / 80.0);
    public static Channel SMALL = new Channel("small", 186.0 / 75.0);
    // Verify against the field and the exact origin used by your autonomous routine.
    public static boolean FIELD_CALIBRATED = false;
    public static boolean START_POSE_CONFIRMED = false;
    public static double START_X = 0, START_Y = 0, START_HEADING_DEG = 0;
    // CAD X/Y (mm) -> field inches: x=72+CAD_X/25.4, y=72+CAD_Y/25.4.
    // Initial estimates from RAISED front-rim envelopes; verify actual aim point on the field.
    // Opposite ends use mirror symmetry. Z is metadata; shot tables calibrate this fixed height.
    public static double[] RED_AUDIENCE = {59.2584, 56.3087, 58.5266};
    public static double[] RED_SCORING = {59.2584, 87.6913, 58.5266};
    public static double[] BLUE_AUDIENCE = {84.7584, 56.3087, 58.5266};
    public static double[] BLUE_SCORING = {84.7584, 87.6913, 58.5266};
    static {
        // Planned wiring: external encoders use the unused encoder sockets of these drive ports.
        // HardwareMap names are the EXISTING motor names, not extra motor configurations.
        LARGE.turret.encoderName = "frontLeft";
        SMALL.turret.encoderName = "backLeft";
        // CAD chassis wheel-center origin (225,130) mm, forward=-CAD_Y, left=-CAD_X.
        // Re-measure relative to the actual Pedro/Pinpoint localization origin.
        LARGE.forwardInches = 1.9745; LARGE.leftInches = 3.7992;
        SMALL.forwardInches = -2.6184; SMALL.leftInches = -4.2323;
    }
    public static double MAX_SHOT_TRANSLATION_IPS = 2.0;
    public static double MAX_SHOT_ROTATION_DEG_S = 5.0;
    public static double READY_DWELL_S = 0.15;

    public static class Channel {
        public String name;
        public TurretConfig turret = new TurretConfig();
        public HoodConfig hood = new HoodConfig();
        public FlywheelConfig flywheel = new FlywheelConfig();
        public FeederConfig feeder = new FeederConfig();
        public boolean mountCalibrated = false;
        // From Pedro localization origin: forward+, left+. Zero yaw shoots robot-forward.
        public double forwardInches = 0, leftInches = 0, yawOffsetDeg = 0;
        // Radial distance from turret pivot to ball release, along shot direction.
        public double muzzleForwardInches = 0;
        // Measured rows: {horizontal release-to-mouth inches, flywheel RPM, hood degrees}.
        // One raised-mouth height only. No extrapolation or invented ballistic calibration.
        public double[][] shotTable = new double[0][3];
        public Channel(String name, double encoderRatio) {
            this.name = name;
            turret.servoName = name + "Turret";
            turret.encoderName = name + "TurretEncoder";
            turret.limitName = name + "TurretHome";
            turret.encoderTurnsPerTurretTurn = encoderRatio;
            hood.servoName = name + "Hood";
            flywheel.motorName = name + "Shooter";
            feeder.hardwareName = name + "Feeder";
        }
    }
    public static class TurretConfig {
        public String servoName, encoderName, limitName;
        // Both CAD ratios assume an encoder on the final pinion shaft; verify each physically.
        public double encoderTurnsPerTurretTurn = 1, ticksPerEncoderTurn = 8192;
        public int encoderSign = 1, servoSign = 1;
        public boolean calibrated = false, limitActiveLow = true;
        // This implementation requires the home magnet at the MINIMUM travel endpoint.
        public double minDeg = -150, maxDeg = 150;
        public double kP = 0.012, kD = 0.001, maxPower = 0.35;
        public double homePower = 0.12, homeTimeoutS = 6, debounceS = 0.04;
        public double toleranceDeg = 1.5, maxReadyVelocityDegS = 8;
        public double stallTimeoutS = 0.6, minimumMotionDeg = 0.2;
        public double maxMeasuredVelocityDegS = 720;
    }
    public static class HoodConfig {
        public String servoName;
        public boolean calibrated = false;
        public double minDeg = 20, maxDeg = 65;
        // Calibrated mapping includes the hood gear ratio and mounting direction.
        public double positionAtMin = 0.2, positionAtMax = 0.8;
        public double settleS = 0.3, secondsPerDegree = 0.015;
    }
    public static class FlywheelConfig {
        public String motorName;
        public boolean reversed = false;
        public double ticksPerMotorTurn = 28, motorTurnsPerWheelTurn = 1;
        public double maxRpm = 6000, kV = 1.0 / 6000, kP = 0.0003;
        public double rpmTolerance = 75, stableS = 0.15;
        public double tuneRpm = 2500;
    }
    public static class FeederConfig {
        public String hardwareName;
        // DISABLED until the mechanism is finalized; alternatively SERVO or MOTOR.
        public String mode = "DISABLED";
        public boolean calibrated = false, reversed = false;
        public double closedPosition = 0.2, openPosition = 0.7, motorPower = 0.5;
        public double pulseS = 0.20, recoveryS = 0.35, waitTimeoutS = 4;
    }
}
