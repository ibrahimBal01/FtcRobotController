package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ShootingConfig.HoodConfig;
import org.firstinspires.ftc.teamcode.aiming.HoodController;

/** Position servo: readiness is a conservative time estimate, not measured position. */
public class Hood extends SubsystemBase {
    private final Servo servo;
    private final HoodController controller;
    public Hood(HardwareMap h, HoodConfig config) {
        servo=h.get(Servo.class,config.servoName);
        controller=new HoodController(config);
    }
    public boolean setAngle(double degrees) {
        double position=controller.request(degrees,Turret.now());
        if (!Double.isFinite(position)) return false;
        servo.setPosition(position);
        return true;
    }
    public boolean atTarget() { return controller.atTarget(Turret.now()); }
    public double targetDegrees() { return controller.targetDegrees(); }
    public void stop() { controller.stop(); }
}

