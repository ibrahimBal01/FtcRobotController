package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.IntakeConfig;

/** Single motor intake: latched on/off, no encoder or ball sensor required. */
public class Intake extends SubsystemBase {
    private final DcMotor motor;
    private boolean running;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, IntakeConfig.MOTOR_NAME);
        motor.setPower(0);
        motor.setDirection(IntakeConfig.REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void start() { running = true; periodic(); }
    public void toggle() { if (running) stop(); else start(); }
    public void stop() { running = false; motor.setPower(0); }
    public boolean isRunning() { return running; }

    @Override public void periodic() {
        double power = IntakeConfig.POWER;
        if (!Double.isFinite(power) || power <= 0 || power > 1) {
            stop();
            return;
        }
        motor.setPower(running ? power : 0);
    }
}
