package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.NectarTurret;

public class Robot {
    public final Drive drive;
    public final Arm arm;
    public final NectarTurret nectarTurret;

    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        arm = new Arm(hardwareMap);
        nectarTurret = new NectarTurret(hardwareMap);
    }
}
