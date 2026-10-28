package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ShootingConfig;
import org.firstinspires.ftc.teamcode.ShootingConfig.FlywheelConfig;
import org.firstinspires.ftc.teamcode.aiming.AimMath;

/** One motor per ball size. RPM means flywheel output RPM. */
public class Shooter extends SubsystemBase {
    private final DcMotorEx motor;
    private final VoltageSensor battery;
    private final FlywheelConfig c;
    private double target, rpm, power, stableSince=Double.NaN, sampleTime=Double.NEGATIVE_INFINITY;
    public Shooter(HardwareMap h) { this(h,ShootingConfig.SMALL.flywheel); }
    public Shooter(HardwareMap h, FlywheelConfig config) {
        c=config; motor=h.get(DcMotorEx.class,c.motorName);
        motor.setDirection(c.reversed?DcMotorSimple.Direction.REVERSE:DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        battery=h.voltageSensor.iterator().next(); stop();
    }
    private boolean valid() {
        return AimMath.finite(c.ticksPerMotorTurn,c.motorTurnsPerWheelTurn,c.maxRpm,c.kP,c.kV,c.rpmTolerance,c.stableS)
                && c.ticksPerMotorTurn>0 && c.motorTurnsPerWheelTurn>0 && c.maxRpm>0
                && c.kP>=0 && c.kV>=0 && c.rpmTolerance>0 && c.stableS>0;
    }
    public void setTargetRpm(double value) {
        if (!valid() || !Double.isFinite(value) || value<=0 || value>c.maxRpm) { stop(); return; }
        if (Math.abs(value-target)>c.rpmTolerance*0.5) stableSince=Double.NaN;
        target=value;
    }
    @Override public void periodic() {
        sampleTime=Turret.now();
        rpm=motor.getVelocity()*60/(c.ticksPerMotorTurn*c.motorTurnsPerWheelTurn);
        double voltage=battery.getVoltage();
        if (!valid() || !AimMath.finite(rpm,voltage) || voltage<5 || target<=0 || target>c.maxRpm) { stop(); return; }
        power=AimMath.clamp(c.kV*target*12/voltage+c.kP*(target-rpm),0,1);
        motor.setPower(power);
        if (Math.abs(target-rpm)>c.rpmTolerance) stableSince=Double.NaN;
        else if (!Double.isFinite(stableSince)) stableSince=sampleTime;
    }
    public void stop() { target=0; power=0; stableSince=Double.NaN; motor.setPower(0); }
    public void invalidateReadiness() { stableSince=Double.NaN; }
    public boolean atSpeed() { return valid() && target>0 && Double.isFinite(stableSince)
            && Turret.now()-sampleTime<0.15 && Math.abs(target-rpm)<=c.rpmTolerance
            && Turret.now()-stableSince>=c.stableS; }
    public boolean readyToFeed() { return atSpeed(); }
    public double getTargetRpm() { return target; }
    public double getRpm() { return rpm; }
    public double getRawRpm() { return rpm; }
    public double getPower() { return power; }
    public void addTelemetry(Telemetry t) {
        t.addData(c.motorName+" target RPM",target); t.addData(c.motorName+" RPM",rpm);
        t.addData(c.motorName+" power",power); t.addData(c.motorName+" at speed",atSpeed());
    }
}

