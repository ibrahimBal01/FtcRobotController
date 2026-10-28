package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TurretConfig;

public class Turret extends SubsystemBase {

    private final String name;
    private final TurretConfig cfg;
    private final CRServo motor;
    private final DcMotorEx encoder;
    private final VoltageSensor battery;
    private final PIDController pid = new PIDController(0,0,0);
    private final ElapsedTime stableTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage;

    private double angleOffsetDegrees = 0;
    private double lastPower = 0;
    private double targetAngle;



    public Turret(HardwareMap hardwareMap, String name, TurretConfig config) {
        this.name = name;
        this.cfg = config;

        motor = hardwareMap.get(CRServo.class, cfg.getMotorName());
        motor.setDirection(cfg.getReversed() ?
                CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);

        encoder = hardwareMap.get(DcMotorEx.class, cfg.getEncoderName());
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();
    }

    @Override
    public void periodic() {
        double measured = getAngleDegrees();
        double power;

        double error = targetAngle - measured;
        if(Math.abs(error) <= cfg.TOLERANCE_DEGREES){
            pid.reset();
            power = 0;
        } else {
            pid.setPID(cfg.kP, cfg.kI, cfg.kD);
            power = pid.calculate(measured, targetAngle) + Math.signum(error) * cfg.kS;
        }

        if ((measured <= cfg.MIN_DEGREES && power < 0) || (measured >= cfg.MAX_DEGREES && power > 0)) {
            power = 0;
        }

        lastPower = Math.max(-1, Math.min(1, power));
        motor.setPower(lastPower);
    }



    public double getAngleDegrees(){
        double encoderRevs = cfg.ENCODER_SIGN * encoder.getCurrentPosition() / cfg.TICKS_PER_ENCODER_REV;
        return encoderRevs / cfg.ENCODER_REVS_PER_TURRET_REV * 360.0 + angleOffsetDegrees;
    }

    public void setTargetAngle(double degrees) {
        targetAngle = Math.max(cfg.MIN_DEGREES, Math.min(cfg.MAX_DEGREES, degrees));
    }

    public boolean atTarget() {
        return Math.abs(targetAngle - getAngleDegrees()) <= cfg.TOLERANCE_DEGREES;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData( name + "Turret angle", getAngleDegrees());
        telemetry.addData(name + "Turret target", targetAngle);
        telemetry.addData(name + "Turret power", lastPower);
        telemetry.addData(name + "Turret On Target", atTarget());
    }

    public void stop(){ motor.setPower(0); }

}