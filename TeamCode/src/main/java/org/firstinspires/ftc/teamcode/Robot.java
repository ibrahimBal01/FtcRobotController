package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.NectarTurret;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Robot {
    public final Drive drive;
    public final Shooter shooter;

    public Telemetry telemetry;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        this.telemetry = telemetry;
    }
}
