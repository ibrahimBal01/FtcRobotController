package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

public final class Constants {
    private Constants() {}

    public static final class Drive {
        public static final double MAX_POWER = 1.0;
        private Drive() {}
    }

    // Non-final fields in here show up in FTC Dashboard's Configuration tab (group "Shooter").
    // Dashboard edits are live only; copy the final values back into this file.
    @Config("Shooter")
    public static final class Shooter {
        // Hardware config names (change to match your Robot Configuration)
        public static final String LEFT_MOTOR = "shooterLeft";
        public static final String RIGHT_MOTOR = "shooterRight";

        // The two motors face each other across the flywheel, so one is usually reversed.
        // If the flywheel fights itself or spins the wrong way, flip these.
        public static final boolean LEFT_REVERSED = false;
        public static final boolean RIGHT_REVERSED = true;

        // goBILDA 6000 rpm (1:1) motor: 28 encoder ticks per output revolution.
        // Change if there is a gear/belt ratio between the encoder and the flywheel.
        public static final double TICKS_PER_REV = 28.0;
        public static final double MAX_RPM = 6000.0;

        // Tunable (non-final so they can be changed live with a dashboard later).
        public static double SHOOT_RPM = 3000.0; // placeholder -- replace with your measured shooting speed

        // Feedforward: motor power per RPM of target. 1/MAX_RPM is the theoretical start value.
        public static double kV = 1.0 / MAX_RPM;
        // Feedback on RPM error (power per RPM of error).
        public static double kP = 0.0009;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double INTEGRAL_LIMIT = 300.0; // clamps accumulated error (rpm*s)

        // Battery voltage the feedforward was tuned at (it is scaled by NOMINAL / actual).
        public static double NOMINAL_VOLTAGE = 12.0;
        // Set false to test whether voltage compensation is causing oscillation.
        public static boolean USE_VOLTAGE_COMP = true;
        // Battery voltage low-pass (1.0 = no filtering). Voltage sags when the motors draw current.
        public static double VOLTAGE_FILTER_ALPHA = 0.1;

        // "At speed" = within RPM_TOLERANCE of target continuously for STABLE_MS.
        public static double RPM_TOLERANCE = 50.0;
        public static double STABLE_MS = 100.0;

        // Recovery boost: if the flywheel is more than RECOVERY_THRESHOLD_RPM below target (after a
        // shot, or during spin-up) apply RECOVERY_POWER instead of the PIDF output, then hand back
        // to PIDF once it is close again. Lower the threshold/power if it overshoots.
        public static boolean USE_RECOVERY_BOOST = true;
        public static double RECOVERY_THRESHOLD_RPM = 100.0;
        public static double RECOVERY_POWER = 1.0;

        // Feeder gate: feed as soon as RPM is within FEED_TOLERANCE_RPM of target (no settle time),
        // so the next piece goes in right after the dip recovers. Keep it above the RPM noise.
        public static double FEED_TOLERANCE_RPM = 100.0;

        // Velocity low-pass filter: 1.0 = no filtering, smaller = smoother but laggier.
        public static double VELOCITY_FILTER_ALPHA = 0.4;

        private Shooter() {}
    }

    public static final class Feeder {
        public static final String MOTOR = "feeder";
        public static final boolean REVERSED = false;
        public static final double POWER = 1.0;

        public static final int TICKS_PER_MOTOR_REV = 28;

        public static final int GEAR_RATIO = 9;

        private Feeder() {}
    }
}
