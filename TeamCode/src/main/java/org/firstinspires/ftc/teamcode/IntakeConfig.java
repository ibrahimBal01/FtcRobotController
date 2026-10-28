package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class IntakeConfig {
    private IntakeConfig() { }
    public static String MOTOR_NAME = "intake";
    // Name and direction changes require a new INIT.
    public static boolean REVERSED = false;
    // Fixed motor power, not RPM control. Start low for the first physical test.
    public static double POWER = 0.5;
}
