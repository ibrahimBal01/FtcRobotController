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
    @Config("Shooters")
    public static final class Shooters {
        // One motor per shooter, goBILDA 6000 rpm (1:1): 28 ticks/rev, 6000 rpm free speed.
        // Hardware names must match your Robot Configuration. For two motors on one shooter use
        // e.g. new String[]{"largeA", "largeB"} and new boolean[]{false, true}.
        public static ShooterConfig LARGE = new ShooterConfig(
                new String[]{"shooterLarge"}, new boolean[]{false}, 28.0, 6000.0, 3000.0,
                "hoodLarge", false);
        public static ShooterConfig SMALL = new ShooterConfig(
                new String[]{"shooterSmall"}, new boolean[]{false}, 28.0, 6000.0, 3000.0,
                "hoodSmall", false);

        static{
            LARGE.kP = 0.0003;
            SMALL.kP = 0.0003;

            LARGE.kV = 1.0 / 6000;
            SMALL.kV = 1.0 / 6000;
        }

        private Shooters() {}
    }

    @Config("Turrets")
    public static final class Turrets {

        public static TurretConfig LARGE = new TurretConfig(
                "turretLarge", false, "encoderLarge"
        );

        public static TurretConfig SMALL = new TurretConfig(
                "turretSmall", false, "encoderSmall");

        static {
            LARGE.kP = 0.0003;
            SMALL.kP = 0.0003;

            LARGE.kS = 0.0;
            SMALL.kS = 0.0;
        }
    }

    public static final class Feeder {
        public static final String MOTOR = "feeder";
        public static final boolean REVERSED = false;
        public static final double POWER = 1.0;

        public static final int TICKS_PER_MOTOR_REV = 28;

        public static final int GEAR_RATIO = 9;

        public static final String SMALL_FLAP = "small_flap";
        public static final String LARGE_FLAP = "large_flap";

        public static final boolean SMALL_FLAP_REVERSED = false;
        public static final boolean LARGE_FLAP_REVERSED = false;

        public static final int FLAP_OPEN_MS = 150;

        public static final double OPENED_POSITION = 0.5;
        public static final double CLOSED_POSITION = 0.0;



        private Feeder() {}
    }

    public static final class Intake {
        public static final String MOTOR = "intake";
        public static final boolean REVERSED = false;
        public static final double POWER = 0.7;
    }
}
