package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Single drivetrain subsystem backed by the Pedro Pathing Follower.
 * Used for both TeleOp (manual driving) and Autonomous (path following / holding).
 *
 * The follower is updated once per scheduler run in periodic(), so nothing else
 * in the project should call follower.update().
 *
 * Sign convention (same as Pedro's DrivePowers and the Tests procedures):
 * forward + = robot front, lateral + = robot left, turn + = counter-clockwise.
 */
public class Drive extends SubsystemBase {
    private final Follower follower;

    public Drive(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void periodic() {
        follower.update();
    }

    // ---------------- TeleOp ----------------

    public void manualRobotCentric(double forward, double lateral, double turn) {
        follower.manual(scaled(forward, lateral, turn));
    }

    public void manualFieldCentric(double forward, double lateral, double turn) {
        double s = org.firstinspires.ftc.teamcode.Constants.Drive.MAX_POWER;
        follower.manual(ManualDrive.fieldCentric(
                forward * s, lateral * s, turn * s, follower.pose().heading()));
    }

    /** Makes the robot's current facing the new field-centric "forward". */
    public void resetHeading() {
        Pose p = follower.pose();
        follower.setPose(new Pose(p.x(), p.y(), 0));
    }

    public void stop() {
        follower.manual(new DrivePowers(0, 0, 0));
    }

    // ---------------- Autonomous ----------------

    public void follow(Path path) {
        follower.follow(path);
    }

    public void hold(Pose pose) {
        follower.hold(pose);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public boolean atParametricEnd() {
        return follower.atParametricEnd();
    }

    // ---------------- Localization ----------------

    public Pose getPose() {
        return follower.pose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    /** Escape hatch for anything not wrapped here (telemetry, follower.mode(), ...). */
    public Follower getFollower() {
        return follower;
    }

    private static DrivePowers scaled(double forward, double lateral, double turn) {
        double s = org.firstinspires.ftc.teamcode.Constants.Drive.MAX_POWER;
        return new DrivePowers(forward * s, lateral * s, turn * s);
    }
}
