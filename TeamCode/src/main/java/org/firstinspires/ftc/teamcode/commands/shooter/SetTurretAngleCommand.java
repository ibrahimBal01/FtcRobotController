package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Moves the turret to an angle and finishes when it is within tolerance.
 * The turret keeps holding that angle afterwards. Add .withTimeout(ms) as a safety cap.
 */
public class SetTurretAngleCommand extends CommandBase {
    private final Turret turret;
    private final double degrees;

    public SetTurretAngleCommand(Turret turret, double degrees) {
        this.turret = turret;
        this.degrees = degrees;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setTargetAngle(degrees);
    }

    @Override
    public boolean isFinished() {
        return turret.atTarget();
    }
}