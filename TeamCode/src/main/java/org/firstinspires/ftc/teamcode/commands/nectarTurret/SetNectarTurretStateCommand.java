package org.firstinspires.ftc.teamcode.commands.nectarTurret;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.NectarTurret;

public class SetNectarTurretStateCommand extends CommandBase {
    private final NectarTurret nectarTurret;
    private final NectarTurret.State targetState;

    public SetNectarTurretStateCommand(NectarTurret nectarTurret, NectarTurret.State targetState){
        this.nectarTurret = nectarTurret;
        this.targetState = targetState;

        addRequirements(nectarTurret);
    }

    @Override
    public void initialize() {
        nectarTurret.setState(targetState);
    }

}
