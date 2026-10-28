package org.firstinspires.ftc.teamcode.aiming;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.ShootingConfig.Channel;
import org.firstinspires.ftc.teamcode.subsystems.*;

/** One ball-size mechanism; FTCLib owns all subsystem periodic updates. */
public final class ShotChannel {
    public final Channel config;
    public final Turret turret;
    public final Hood hood;
    public final Shooter shooter;
    public final Feeder feeder;
    public ShotChannel(HardwareMap h, Channel config) {
        this.config=config; turret=new Turret(h,config.turret); hood=new Hood(h,config.hood);
        shooter=new Shooter(h,config.flywheel); feeder=new Feeder(h,config.feeder);
    }
    public void stop() { feeder.stop(); shooter.stop(); turret.stop(); hood.stop(); }
}
