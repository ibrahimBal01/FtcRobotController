package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.ShootingConfig.TurretConfig;
import org.firstinspires.ftc.teamcode.aiming.TurretController;

public class Turret extends SubsystemBase {
    private final CRServo servo;
    private final DcMotorEx encoder;
    private final DigitalChannel homeSwitch;
    private final TurretConfig config;
    private final TurretController controller;
    public Turret(HardwareMap hardware, TurretConfig config) {
        this.config=config;
        servo=hardware.get(CRServo.class,config.servoName);
        encoder=hardware.get(DcMotorEx.class,config.encoderName);
        homeSwitch=hardware.get(DigitalChannel.class,config.limitName);
        homeSwitch.setMode(DigitalChannel.Mode.INPUT);
        controller=new TurretController(config); servo.setPower(0);
        // Read-only encoder port: never reset it, change motor mode, or write motor power.
    }
    public static double now() { return System.nanoTime()*1e-9; }
    public boolean homeSwitchActive() { return homeSwitch.getState() != config.limitActiveLow; }
    public void home() { controller.home(now(),homeSwitchActive()); }
    public boolean setTargetDegrees(double angle) { return controller.aim(angle); }
    @Override public void periodic() {
        servo.setPower(controller.update(encoder.getCurrentPosition(),homeSwitchActive(),now())*config.servoSign);
    }
    public void stop() { controller.stop(); servo.setPower(0); }
    public boolean atTarget() { return controller.atTarget(); }
    public double angleDegrees() { return controller.angle(); }
    public TurretController.State state() { return controller.state(); }
    public String fault() { return controller.fault(); }
}
