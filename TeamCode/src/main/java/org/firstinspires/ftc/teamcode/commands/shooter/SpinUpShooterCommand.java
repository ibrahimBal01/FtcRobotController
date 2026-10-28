package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Sets the target rpm and finishes once the flywheel is at speed (Shooter.atSpeed() already
 * requires it to stay within tolerance for a short time), so it can gate a feeder in a
 * SequentialCommandGroup. The flywheel keeps spinning after the command ends; use
 * StopShooterCommand to stop it. Add .withTimeout(ms) as a safety cap.
 */
public class SpinUpShooterCommand extends CommandBase {
    private final Shooter shooter;
    private final double rpm;

    public SpinUpShooterCommand(Shooter shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetRpm(rpm);
    }

    @Override
    public boolean isFinished() {
        return shooter.atSpeed();
    }

    @Override public void end(boolean interrupted) {
        if (interrupted) shooter.stop();
    }
}
