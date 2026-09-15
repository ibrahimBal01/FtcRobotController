package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.arm.ScoreSequence;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

@Autonomous(name = "Simple Auto", group = "Competition")
public class SimpleAuto extends CommandOpMode {

    private Robot robot;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);

        schedule(new SequentialCommandGroup(
                new SetArmStateCommand(robot.arm, Arm.State.INTAKE),
                new WaitCommand(500),
                new ScoreSequence(robot.arm),
                new WaitCommand(500),
                new SetArmStateCommand(robot.arm, Arm.State.HOME)
        ));
    }
}
