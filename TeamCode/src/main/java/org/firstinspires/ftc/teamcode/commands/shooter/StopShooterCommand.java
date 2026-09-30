package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class StopShooterCommand extends InstantCommand {

    public StopShooterCommand(Shooter shooter) {
        super(shooter::stop, shooter);
    }
}