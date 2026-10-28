package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

/** Open-loop feeder that pushes game pieces into the shooter flywheel. */
public class Feeder extends SubsystemBase {
    private final DcMotorEx motor;
    private final Servo small_flap;
    private final Servo large_flap;


    public Feeder(HardwareMap hardwareMap) {
        small_flap = hardwareMap.get(Servo.class, Constants.Feeder.SMALL_FLAP);
        large_flap = hardwareMap.get(Servo.class, Constants.Feeder.LARGE_FLAP);

        small_flap.setDirection(Constants.Feeder.SMALL_FLAP_REVERSED ?
                Servo.Direction.REVERSE: Servo.Direction.FORWARD);

        large_flap.setDirection(Constants.Feeder.LARGE_FLAP_REVERSED ?
                Servo.Direction.REVERSE: Servo.Direction.FORWARD);

        motor = hardwareMap.get(DcMotorEx.class, Constants.Feeder.MOTOR);

        motor.setDirection(Constants.Feeder.REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        closeLargeFlap();
        closeSmallFlap();

        stop();
    }

    public void run() {
        motor.setPower(Constants.Feeder.POWER);
    }
    public void setPower(double power) {
        motor.setPower(power);
    }

    public void openSmallFlap() { small_flap.setPosition(0.5); }
    public void closeSmallFlap() { small_flap.setPosition(0.0); }
    public void openLargeFlap() { large_flap.setPosition(0.5); }
    public void closeLargeFlap() { large_flap.setPosition(0.0); }

    public void stop() {
        motor.setPower(0);
    }

}
