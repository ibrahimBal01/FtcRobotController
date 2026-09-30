package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

/**
 * TeleOp driving through the Pedro Follower.
 * Suppliers use Pedro's convention: forward +, lateral + = left, turn + = counter-clockwise.
 */
public class DefaultDriveCommand extends CommandBase {
    private final Drive drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier lateral;
    private final DoubleSupplier turn;
    private final boolean fieldCentric;

    public DefaultDriveCommand(
            Drive drive,
            DoubleSupplier forward,
            DoubleSupplier lateral,
            DoubleSupplier turn,
            boolean fieldCentric) {

        this.drive = drive;
        this.forward = forward;
        this.lateral = lateral;
        this.turn = turn;
        this.fieldCentric = fieldCentric;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (fieldCentric) {
            drive.manualFieldCentric(forward.getAsDouble(), lateral.getAsDouble(), turn.getAsDouble());
        } else {
            drive.manualRobotCentric(forward.getAsDouble(), lateral.getAsDouble(), turn.getAsDouble());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
