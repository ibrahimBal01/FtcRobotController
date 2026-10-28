package org.firstinspires.ftc.teamcode.commands.shooter;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.aiming.*;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/** Owns feeder only; tracking command owns turret/hood/flywheel. Bind with whenPressed. */
public class ShootOneCommand extends CommandBase {
    private final ShotChannel channel;
    private final TrackTargetCommand tracking;
    private final FeedCycle cycle=new FeedCycle();
    private boolean opened;
    public ShootOneCommand(ShotChannel channel,TrackTargetCommand tracking) {
        this.channel=channel; this.tracking=tracking; addRequirements(channel.feeder);
    }
    @Override public void initialize() { channel.feeder.stop(); opened=false; cycle.start(Turret.now()); }
    @Override public void execute() {
        boolean fire=cycle.update(Turret.now(),tracking.readyToFire() && channel.feeder.readyForPulse(),
                tracking.safeToContinueFeed() && channel.feeder.available(),
                channel.config.feeder.waitTimeoutS,channel.config.feeder.pulseS);
        if(fire) { channel.feeder.run(); opened=true; }
        else channel.feeder.stop();
    }
    @Override public boolean isFinished() { return cycle.finished() || !channel.feeder.available(); }
    @Override public void end(boolean interrupted) {
        channel.feeder.stop();
        if(opened) { channel.feeder.finishPulse(); channel.shooter.invalidateReadiness(); }
    }
}

