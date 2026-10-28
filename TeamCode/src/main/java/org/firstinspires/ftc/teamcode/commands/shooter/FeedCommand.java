package org.firstinspires.ftc.teamcode.commands.shooter;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import org.firstinspires.ftc.teamcode.aiming.FeedCycle;
import org.firstinspires.ftc.teamcode.subsystems.*;

/** Bounded BENCH calibration pulse. Competition uses ShootOneCommand with field tracking. */
public class FeedCommand extends CommandBase {
    private final Feeder feeder;
    private final Shooter shooter;
    private final BooleanSupplier alignment;
    private final FeedCycle cycle=new FeedCycle();
    private boolean opened;
    public FeedCommand(Feeder feeder,Shooter shooter) { this(feeder,shooter,()->true); }
    public FeedCommand(Feeder feeder,Shooter shooter,BooleanSupplier alignment) {
        if(shooter==null) throw new IllegalArgumentException("Shooter required");
        this.feeder=feeder;this.shooter=shooter;this.alignment=alignment;addRequirements(feeder);
    }
    @Override public void initialize() { opened=false;feeder.stop();cycle.start(Turret.now()); }
    @Override public void execute() {
        boolean safe=alignment.getAsBoolean() && shooter.getTargetRpm()>0 && feeder.available();
        if(cycle.update(Turret.now(),safe && shooter.atSpeed() && feeder.readyForPulse(),safe,
                feeder.config().waitTimeoutS,feeder.config().pulseS)) {
            feeder.run();opened=true;
        } else feeder.stop();
    }
    @Override public boolean isFinished() { return cycle.finished() || !feeder.available(); }
    @Override public void end(boolean interrupted) {
        feeder.stop();
        if(opened) {feeder.finishPulse();shooter.invalidateReadiness();}
    }
}

