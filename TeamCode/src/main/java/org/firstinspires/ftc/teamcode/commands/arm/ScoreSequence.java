package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ScoreSequence extends SequentialCommandGroup {

    public ScoreSequence(Arm arm) {
        addCommands(
                new SetArmStateCommand(arm, Arm.State.TRANSFER),
                new SetArmStateCommand(arm, Arm.State.SCORE)
        );
    }
}
