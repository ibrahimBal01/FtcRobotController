package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class DefaultDriveCommand extends CommandBase {
    private final Drive drive;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier rotation;

    public DefaultDriveCommand(
            Drive drive,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rotation) {

        this.drive = drive;
        this.x = x;
        this.y = y;
        this.rotation = rotation;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.driveFieldRelative(
                x.getAsDouble(),
                y.getAsDouble(),
                rotation.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
