package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class SetShooterRPMCommand extends CommandBase {

    private final Shooter shooter;
    private final double targetRPM;
    private final double toleranceRPM;

    public SetShooterRPMCommand(
            Shooter shooter,
            double targetRPM,
            double toleranceRPM
    ) {
        this.shooter = shooter;
        this.targetRPM = targetRPM;
        this.toleranceRPM = toleranceRPM;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setRPM(targetRPM);
    }

    @Override
    public boolean isFinished() {
        return shooter.atTargetRPM(toleranceRPM);
    }
}