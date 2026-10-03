package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/** Sets the flywheel target RPM and finishes immediately; the flywheel keeps spinning. */
public class SetShooterRpmCommand extends CommandBase {
    private final Shooter shooter;
    private final double rpm;

    public SetShooterRpmCommand(Shooter shooter, double rpm) {
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
        return true;
    }
}
