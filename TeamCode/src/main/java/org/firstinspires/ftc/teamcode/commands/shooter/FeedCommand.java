package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Runs the feeder until cancelled (e.g. whileHeld, or .withTimeout(ms) in auto).
 *
 * With no shooters given the feeder just runs. With shooters given, it only runs while every
 * shooter that is currently spinning (target > 0) is within its feedToleranceRpm of target, and
 * at least one is spinning. So pass both shooters and the feeder follows whichever one you
 * spun up; it pauses while the flywheel recovers after a shot.
 */
public class FeedCommand extends CommandBase {
    private final Feeder feeder;
    private final Shooter large, small;
    private final ElapsedTime flapTimer = new ElapsedTime();
    private boolean flapsOpen = false;

    public FeedCommand(Feeder feeder, Shooter large, Shooter small) {
        this.feeder = feeder;
        this.large = large;
        this.small = small;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        flapsOpen = false;
    }

    @Override
    public void execute() {
        boolean largeOn = large.getTargetRpm() > 0;
        boolean smallOn = small.getTargetRpm() > 0;

        // Open only the flap of a shooter that is spinning
        if (largeOn) feeder.openLargeFlap(); else feeder.closeLargeFlap();
        if (smallOn) feeder.openSmallFlap(); else feeder.closeSmallFlap();

        boolean anyOn = largeOn || smallOn;
        if (anyOn && !flapsOpen) flapTimer.reset();   // start the open delay
        flapsOpen = anyOn;

        boolean ready = (!largeOn || large.readyToFeed())
                && (!smallOn || small.readyToFeed());

        if (anyOn && ready && flapTimer.milliseconds() >= Constants.Feeder.FLAP_OPEN_MS) {
            feeder.run();
        } else {
            feeder.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        feeder.closeLargeFlap();
        feeder.closeSmallFlap();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
