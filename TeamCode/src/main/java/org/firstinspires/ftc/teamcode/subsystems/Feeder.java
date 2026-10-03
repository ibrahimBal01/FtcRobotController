package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

/** Open-loop feeder that pushes game pieces into the shooter flywheel. */
public class Feeder extends SubsystemBase {
    private final DcMotorEx motor;

    public Feeder(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, Constants.Feeder.MOTOR);
        motor.setDirection(Constants.Feeder.REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    public void run() {
        motor.setPower(Constants.Feeder.POWER);
    }
    public void setPower(double power) {
        motor.setPower(power);
    }

    public void stop() {
        motor.setPower(0);
    }

    public double getRpm() {
        double motorRpm = motor.getVelocity() * 60.0 / Constants.Feeder.TICKS_PER_MOTOR_REV;
        return motorRpm / Constants.Feeder.GEAR_RATIO;
    }
}
