package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Arm extends SubsystemBase {

    public enum State {
        HOME,
        INTAKE,
        TRANSFER,
        SCORE
    }

    private final DcMotorEx motor;
    private State state = State.HOME;

    public Arm(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "arm");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setState(State newState) {
        state = newState;
    }

    public State getState() {
        return state;
    }

    @Override
    public void periodic() {
        // The FSM lives here.
        // Commands choose the desired state; the subsystem owns the behavior.

        int target;

        switch (state) {
            case HOME:
                target = Constants.Arm.HOME_TICKS;
                break;

            case INTAKE:
                target = Constants.Arm.INTAKE_TICKS;
                break;

            case TRANSFER:
                target = Constants.Arm.TRANSFER_TICKS;
                break;

            case SCORE:
                target = Constants.Arm.SCORE_TICKS;
                break;

            default:
                target = Constants.Arm.HOME_TICKS;
        }

        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Constants.Arm.POWER);
    }

    public boolean atTarget() {
        return !motor.isBusy();
    }

    public void stop() {
        motor.setPower(0);
    }
}
