package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class Robot {
    public final Drive drive;
    public final Arm arm;

    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        arm = new Arm(hardwareMap);
    }
}
