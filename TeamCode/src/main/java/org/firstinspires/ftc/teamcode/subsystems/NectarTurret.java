package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class NectarTurret extends SubsystemBase {

    public enum State{
        DEFAULT,
        SCORE
    }

    private final Servo turretServo;
    private State state = State.DEFAULT;

    public NectarTurret(HardwareMap hardwareMap){
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretServo.setDirection(Servo.Direction.FORWARD);
        turretServo.setPosition(0);

    }

    @Override
    public void periodic() {
        int targetPosition;

        switch (state){
            case DEFAULT:
                targetPosition = 0;
                break;

            case SCORE:
                targetPosition = 60;
                break;

            default:
                targetPosition = 0;
        }

        turretServo.setPosition(targetPosition);

    }

    public void setState(State newState){
        state = newState;
    }

    public State getState(){
        return state;
    }

    public void setPosition(int position){
        turretServo.setPosition(position);
    }




}
