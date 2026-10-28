package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.subsystems.Hood;

/** Moves the hood to a servo position (0..1, clamped to the config limits) and finishes. */
public class SetHoodPositionCommand extends CommandBase {
    private final Hood hood;
    private final DoubleSupplier position;

    public SetHoodPositionCommand(Hood hood, double position) {
        this(hood, () -> position);
    }

    /** Position is read when the command starts (e.g. () -> Constants.Shooters.LARGE.hoodPosition). */
    public SetHoodPositionCommand(Hood hood, DoubleSupplier position) {
        this.hood = hood;
        this.position = position;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setPosition(position.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
