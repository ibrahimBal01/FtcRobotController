package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

/**
 * Drives to a pose and holds it. Finishes when the follower reports it is settled.
 * Add .withTimeout(ms) if you want a hard cap.
 */
public class HoldPointCommand extends CommandBase {
    private final Drive drive;
    private final Pose pose;
    private boolean executed;

    public HoldPointCommand(Drive drive, Pose pose) {
        this.drive = drive;
        this.pose = pose;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        executed = false;
        drive.hold(pose);
    }

    @Override
    public void execute() {
        executed = true;
    }

    @Override
    public boolean isFinished() {
        return executed && !drive.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) drive.stop();
    }
}
