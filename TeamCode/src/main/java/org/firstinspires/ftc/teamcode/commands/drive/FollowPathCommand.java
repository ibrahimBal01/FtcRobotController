package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.Path;

import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

/**
 * Follows a Pedro path. Finishes when the follower is no longer busy (settled at the end),
 * or, with waitForStop = false, as soon as the robot reaches the parametric end of the path
 * (faster chaining, less precise). Add .withTimeout(ms) if you want a hard cap.
 */
public class FollowPathCommand extends CommandBase {
    private final Drive drive;
    private final Supplier<Path> pathSupplier;
    private final boolean waitForStop;
    private boolean executed;

    public FollowPathCommand(Drive drive, Path path) {
        this(drive, () -> path, true);
    }

    public FollowPathCommand(Drive drive, Path path, boolean waitForStop) {
        this(drive, () -> path, waitForStop);
    }

    /** Path is built when the command starts (useful for paths that depend on the current pose). */
    public FollowPathCommand(Drive drive, Supplier<Path> pathSupplier, boolean waitForStop) {
        this.drive = drive;
        this.pathSupplier = pathSupplier;
        this.waitForStop = waitForStop;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        executed = false;
        drive.follow(pathSupplier.get());
    }

    @Override
    public void execute() {
        executed = true; // guarantees at least one follower update before we check isBusy()
    }

    @Override
    public boolean isFinished() {
        if (!executed) return false;
        return waitForStop ? !drive.isBusy() : drive.atParametricEnd();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) drive.stop();
    }
}
