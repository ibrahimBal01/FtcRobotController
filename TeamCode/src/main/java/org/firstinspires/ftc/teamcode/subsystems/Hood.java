package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ShooterConfig;

/**
 * Adjustable hood (shot angle) driven by a position servo. One instance per shooter; it is a
 * separate subsystem from Shooter so hood commands never interrupt flywheel commands.
 *
 * A position servo has no feedback, so getPosition() is the last position commanded.
 * Commanded positions are clamped to the config's hoodMinPosition..hoodMaxPosition.
 */
public class Hood extends SubsystemBase {
    private final String name;
    private final ShooterConfig cfg;
    private final Servo servo;
    private double position;
    private boolean initialized = false;

    /** @param name short label used in telemetry keys, e.g. "Large" */
    public Hood(HardwareMap hardwareMap, String name, ShooterConfig config) {
        this.name = name;
        this.cfg = config;
        servo = hardwareMap.get(Servo.class, cfg.hoodServoName());
        servo.setDirection(cfg.hoodReversed() ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        setPosition(cfg.hoodPosition); // start at the configured default
    }

    public void setPosition(double newPosition) {
        double clamped = Math.max(cfg.hoodMinPosition, Math.min(cfg.hoodMaxPosition, newPosition));
        if (clamped != position || !initialized) {
            position = clamped;
            servo.setPosition(position);
            initialized = true;
        }
    }

    /** Moves the hood by delta (servo position units), staying within the limits. */
    public void adjust(double delta) {
        setPosition(position + delta);
    }

    public double getPosition() {
        return position;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData(name + " hood position", position);
    }

}
