package org.firstinspires.ftc.teamcode.commands.shooter;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.ShootingConfig;
import org.firstinspires.ftc.teamcode.aiming.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

/** Tracks the chosen raised mouth. Drive is only read, so manual/path drive can run concurrently. */
public class TrackTargetCommand extends CommandBase {
    private final ShotChannel ch;
    private final Drive drive;
    private final FieldTargets targets;
    private boolean active, valid, stationary, ready;
    private double lastTick, stableSince=Double.NaN;
    private final MotionGate motion=new MotionGate();
    private double[] targetSnapshot;
    private double distance=Double.NaN;
    private String status="Stopped";
    public TrackTargetCommand(Drive drive, ShotChannel channel, FieldTargets targets) {
        this.drive=drive; ch=channel; this.targets=targets;
        addRequirements(ch.turret,ch.hood,ch.shooter);
    }
    @Override public void initialize() {
        active=true; valid=false; ready=false; motion.reset(); stableSince=Double.NaN;
    }
    @Override public void execute() {
        lastTick=Turret.now(); valid=false; ready=false; stationary=false;
        Pose pose=drive.getPose();
        double sampleTime=drive.getPoseUpdateTime();
        stationary=motion.update(pose.x(),pose.y(),pose.heading(),sampleTime,lastTick,
                ShootingConfig.MAX_SHOT_TRANSLATION_IPS,ShootingConfig.MAX_SHOT_ROTATION_DEG_S);
        targetSnapshot=targets.point();
        if (!ShootingConfig.FIELD_CALIBRATED || !drive.isFieldPoseConfirmed() || !ch.config.mountCalibrated
                || !drive.isLocalizationReady()) {
            invalidate("Field / initial pose / mount / Pinpoint not ready"); return;
        }
        if (targetSnapshot==null || targetSnapshot.length!=3 || !AimMath.finite(targetSnapshot)
                || !AimMath.finite(pose.x(),pose.y(),pose.heading(),ch.config.forwardInches,
                ch.config.leftInches,ch.config.yawOffsetDeg,ch.config.muzzleForwardInches)
                || lastTick-sampleTime>0.15) { invalidate("Invalid/stale pose or target"); return; }
        double[] aim=AimMath.solve(pose.x(),pose.y(),pose.heading(),ch.config.forwardInches,
                ch.config.leftInches,ch.config.yawOffsetDeg,targetSnapshot[0],targetSnapshot[1],
                ch.config.muzzleForwardInches);
        distance=aim[1];
        if (distance<=0 || !targets.onEntranceSide(aim[2],aim[3])) {
            invalidate("Move to the selected mouth's entrance side"); return;
        }
        if (!ch.turret.setTargetDegrees(aim[0])) {
            invalidate("Turret unhomed, faulted or target outside travel"); return;
        }
        double[] shot=ShotTable.lookup(ch.config.shotTable,distance);
        if (shot==null || shot[0]>ch.config.flywheel.maxRpm) {
            ch.shooter.stop(); ch.hood.stop(); stableSince=Double.NaN;
            status="Aiming only: no calibrated shot at this range"; return;
        }
        if (!ch.hood.setAngle(shot[1])) { invalidate("Hood calibration/range required"); return; }
        ch.shooter.setTargetRpm(shot[0]); valid=true;
        boolean aligned=stationary && ch.turret.atTarget() && ch.hood.atTarget() && ch.shooter.atSpeed();
        if (!aligned) stableSince=Double.NaN;
        else if (!Double.isFinite(stableSince)) stableSince=lastTick;
        ready=aligned && Double.isFinite(ShootingConfig.READY_DWELL_S) && ShootingConfig.READY_DWELL_S>0
                && lastTick-stableSince>=ShootingConfig.READY_DWELL_S;
        status=ready?"READY":(!stationary?"Tracking: stop robot to fire":"Aligning / spinning up");
    }
    private void invalidate(String reason) {
        valid=false; ready=false; stableSince=Double.NaN; status=reason;
        ch.turret.stop(); ch.hood.stop(); ch.shooter.stop();
    }
    public boolean safeToContinueFeed() {
        double[] current=targets.point();
        Pose pose=drive.getPose();
        boolean latestStationary=motion.update(pose.x(),pose.y(),pose.heading(),drive.getPoseUpdateTime(),
                Turret.now(),ShootingConfig.MAX_SHOT_TRANSLATION_IPS,ShootingConfig.MAX_SHOT_ROTATION_DEG_S);
        return active && valid && stationary && latestStationary && Turret.now()-lastTick<0.15
                && ShootingConfig.FIELD_CALIBRATED && drive.isFieldPoseConfirmed() && ch.config.mountCalibrated
                && drive.isLocalizationReady()
                && current!=null && java.util.Arrays.equals(current,targetSnapshot)
                && ch.turret.atTarget() && ch.hood.atTarget() && ch.shooter.getTargetRpm()>0;
    }
    public boolean readyToFire() { return ready && safeToContinueFeed() && ch.shooter.atSpeed(); }
    public String status() { return status; }
    public double distanceInches() { return distance; }
    @Override public void end(boolean interrupted) {
        active=false; valid=false; ready=false; ch.stop(); status="Stopped";
    }
}

