package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.ShootingConfig;
import org.firstinspires.ftc.teamcode.ShootingConfig.FeederConfig;
import org.firstinspires.ftc.teamcode.aiming.AimMath;

/** DISABLED needs no hardware and cannot release a ball. Change mode only between OpModes. */
public class Feeder extends SubsystemBase {
    private final FeederConfig c;
    private final String mode;
    private Servo servo;
    private DcMotorEx motor;
    private boolean open;
    private double leaseUntil;
    private double cooldownUntil;
    public Feeder(HardwareMap h) { this(h,ShootingConfig.SMALL.feeder); }
    public Feeder(HardwareMap h, FeederConfig config) {
        c=config; mode=c.mode;
        if ("SERVO".equals(mode)) servo=h.get(Servo.class,c.hardwareName);
        else if ("MOTOR".equals(mode)) {
            motor=h.get(DcMotorEx.class,c.hardwareName);
            motor.setDirection(c.reversed?DcMotorSimple.Direction.REVERSE:DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (!"DISABLED".equals(mode)) throw new IllegalArgumentException("Unknown feeder mode: "+mode);
        stop();
    }
    public boolean available() {
        return c.calibrated && mode.equals(c.mode) && (servo!=null || motor!=null)
                && AimMath.finite(c.closedPosition,c.openPosition,c.motorPower,c.pulseS,c.recoveryS,c.waitTimeoutS)
                && c.closedPosition>=0 && c.closedPosition<=1 && c.openPosition>=0 && c.openPosition<=1
                && c.motorPower>0 && c.motorPower<=1 && c.pulseS>0 && c.pulseS<=1
                && c.recoveryS>=0 && c.waitTimeoutS>0;
    }
    public void run() {
        if (!available()) { stop(); return; }
        if (servo!=null) servo.setPosition(c.openPosition);
        if (motor!=null) motor.setPower(c.motorPower);
        open=true; leaseUntil=Turret.now()+0.15;
    }
    public void stop() {
        if (servo!=null && Double.isFinite(c.closedPosition) && c.closedPosition>=0 && c.closedPosition<=1)
            servo.setPosition(c.closedPosition);
        if (motor!=null) motor.setPower(0);
        open=false;
    }
    @Override public void periodic() { if (open && (!available() || Turret.now()>leaseUntil)) stop(); }
    public boolean isOpen() { return open; }
    public boolean readyForPulse() { return available() && Turret.now() >= cooldownUntil; }
    public void finishPulse() { stop(); cooldownUntil = Turret.now() + c.recoveryS; }
    public FeederConfig config() { return c; }
}

