package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.drive.FollowPathCommand;

import static com.pedropathing.api.Paths.line;

/**
 * Placeholder auto: drives 24 in forward, waits, drives back.
 * Replace the poses/paths with your real routine.
 */
@Autonomous(name = "Simple Auto", group = "Competition")
public class SimpleAuto extends CommandOpMode {

    private static final Pose START = new Pose(0, 0, 0);
    private static final Pose FORWARD = new Pose(24, 0, 0);

    private Robot robot;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);

        Path out = line(START, FORWARD).constant(0);
        Path back = line(FORWARD, START).constant(0);

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> robot.drive.setPose(START)),
                new FollowPathCommand(robot.drive, out),
                new WaitCommand(500),
                new FollowPathCommand(robot.drive, back)
        ));
    }
}
