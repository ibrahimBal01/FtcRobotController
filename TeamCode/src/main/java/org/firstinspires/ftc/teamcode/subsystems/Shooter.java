package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter extends SubsystemBase {

    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final DcMotorEx feeder;

    private final Telemetry telemtry;

    // Change this to whatever RPM you determine is appropriate.
    private double targetRPM = 0;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        this.telemtry = telemetry;

        /*
         * Set directions so both wheels spin in the correct physical
         * direction for your shooter.
         *
         * You may need to swap these depending on how the motors
         * are mounted.
         */
        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stop();
    }

    /**
     * Set the desired shooter speed in RPM.
     */
    public void setRPM(double rpm) {
        targetRPM = rpm;

        double ticksPerSecond = rpmToTicksPerSecond(rpm);

        shooterLeft.setVelocity(ticksPerSecond);
        shooterRight.setVelocity(ticksPerSecond);
    }

    /**
     * Stop both shooter motors.
     */
    public void stop() {
        targetRPM = 0;

        shooterLeft.setVelocity(0);
        shooterRight.setVelocity(0);
    }

    /**
     * Returns the target RPM.
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Returns the current RPM of the left motor.
     */
    public double getLeftRPM() {
        return ticksPerSecondToRPM(shooterLeft.getVelocity());
    }

    /**
     * Returns the current RPM of the right motor.
     */
    public double getRightRPM() {
        return ticksPerSecondToRPM(shooterRight.getVelocity());
    }

    public void setFeederPower(double power){
        feeder.setPower(power);
    }

    /**
     * Returns true if both motors are sufficiently close
     * to the requested RPM.
     */
    public boolean atTargetRPM(double toleranceRPM) {

        return Math.abs(getLeftRPM() - targetRPM) <= toleranceRPM
                && Math.abs(getRightRPM() - targetRPM) <= toleranceRPM;
    }

    private double rpmToTicksPerSecond(double rpm) {

        double leftTicksPerRev =
                shooterLeft.getMotorType().getTicksPerRev();

        return rpm * leftTicksPerRev / 60.0;
    }

    private double ticksPerSecondToRPM(double ticksPerSecond) {

        double ticksPerRev =
                shooterLeft.getMotorType().getTicksPerRev();

        return ticksPerSecond * 60.0 / ticksPerRev;
    }

    @Override
    public void periodic() {
        telemtry.addData("Left Shooter RPM", getLeftRPM());
        telemtry.addData("Right Shooter RPM", getRightRPM());

        telemtry.update();



        // Velocity control maintains the requested RPM automatically.
        // This method can later be used for telemetry.
    }
}