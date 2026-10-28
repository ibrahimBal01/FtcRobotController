package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Robot {
    public final Drive drive;
    public final Shooter large_shooter;

    public final Shooter small_shooter;
    public final Feeder feeder;


    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        large_shooter = new Shooter(hardwareMap, "Large", Constants.Shooters.LARGE);
        small_shooter = new Shooter(hardwareMap, "Small", Constants.Shooters.SMALL);
        feeder = new Feeder(hardwareMap);
    }
}
