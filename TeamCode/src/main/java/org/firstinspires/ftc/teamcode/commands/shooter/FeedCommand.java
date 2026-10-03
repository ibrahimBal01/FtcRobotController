package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Runs the feeder until cancelled (e.g. whileHeld, or .withTimeout(ms) in auto).
 * If a Shooter is given, the feeder only runs while the flywheel is at speed, so every
 * piece is launched at the same RPM; it pauses while the flywheel recovers after a shot.
 */
public class FeedCommand extends CommandBase {
    private final Feeder feeder;
    private final Shooter shooter; // may be null = ungated

    public FeedCommand(Feeder feeder) {
        this(feeder, null);
    }

    public FeedCommand(Feeder feeder, Shooter shooter) {
        this.feeder = feeder;
        this.shooter = shooter;
        addRequirements(feeder); // only reads the shooter, so it does not require it
    }

    @Override
    public void execute() {
        if (shooter == null || shooter.readyToFeed()) {
            feeder.run();
        } else {
            feeder.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
