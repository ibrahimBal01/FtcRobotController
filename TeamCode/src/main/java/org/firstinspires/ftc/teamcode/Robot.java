package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.aiming.FieldTargets;
import org.firstinspires.ftc.teamcode.aiming.ShotChannel;

public class Robot {
    public final Drive drive;
    public final ShotChannel large, small;
    public final FieldTargets targets=new FieldTargets();
    /** Drive-only autonomous/tuning does not require shooting hardware. */
    public Robot(HardwareMap h) { this(h,false); }
    public Robot(HardwareMap h, boolean withShooting) {
        if (withShooting) validateEncoderPorts();
        drive=new Drive(h);
        large=withShooting?new ShotChannel(h,ShootingConfig.LARGE):null;
        small=withShooting?new ShotChannel(h,ShootingConfig.SMALL):null;
    }
    private static void validateEncoderPorts() {
        String a=ShootingConfig.LARGE.turret.encoderName, b=ShootingConfig.SMALL.turret.encoderName;
        if(a.equals(b) || a.equals(ShootingConfig.LARGE.flywheel.motorName)
                || a.equals(ShootingConfig.SMALL.flywheel.motorName)
                || b.equals(ShootingConfig.LARGE.flywheel.motorName)
                || b.equals(ShootingConfig.SMALL.flywheel.motorName))
            throw new IllegalArgumentException("Turrets need separate encoder ports, separate from flywheel encoders");
    }
    public void stop() {
        if(large!=null) large.stop();
        if(small!=null) small.stop();
        drive.stop();
    }
}

