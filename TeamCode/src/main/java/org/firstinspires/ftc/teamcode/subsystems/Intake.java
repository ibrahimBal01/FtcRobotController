package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Intake extends SubsystemBase {
    private final DcMotorEx motor;
    public Intake(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, Constants.Feeder.MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(Constants.Intake.REVERSED ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        stop();
    }

    public void run() { setPower(Constants.Intake.POWER); }
    public void setPower(double power){ motor.setPower(power); }

    public void stop() { setPower(0.0); }
}
