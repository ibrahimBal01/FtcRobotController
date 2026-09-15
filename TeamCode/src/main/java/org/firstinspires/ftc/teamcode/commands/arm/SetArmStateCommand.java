package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class SetArmStateCommand extends CommandBase {
    private final Arm arm;
    private final Arm.State targetState;

    public SetArmStateCommand(Arm arm, Arm.State targetState) {
        this.arm = arm;
        this.targetState = targetState;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setState(targetState);
    }

    @Override
    public boolean isFinished() {
        return arm.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            arm.stop();
        }
    }
}
