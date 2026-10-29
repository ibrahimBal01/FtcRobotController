package org.firstinspires.ftc.teamcode;

public class TurretConfig {

    private final String motorName;
    private final boolean reversed;
    private final String encoderName;

    public double ENCODER_SIGN = 1;
    public double TICKS_PER_ENCODER_REV = 8192;
    public double ENCODER_REVS_PER_TURRET_REV = 1;

    public double TOLERANCE_DEGREES = 1.0;

    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kS = 0.0;
    public double kV = 0.0;

    public double MIN_DEGREES = -100;
    public double MAX_DEGREES = 100;

    public final boolean useVoltageComp = false;

    public final double nominalVoltage = 12.0;




    public TurretConfig(String motorName, boolean reversed, String encoderName){
        this.motorName = motorName;
        this.reversed = reversed;
        this.encoderName = encoderName;
    }

    public String getMotorName() { return motorName; }
    public boolean getReversed() { return reversed; }
    public String getEncoderName() { return encoderName; }


}
