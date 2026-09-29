package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Simple Auto", group = "Competition")
public class SimpleAuto extends CommandOpMode {

    private Robot robot;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);

        schedule(new SequentialCommandGroup(
        ));
    }
}
