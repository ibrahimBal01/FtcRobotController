package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

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
    private final GoBildaPinpointDriver pinpoint;
    private double fieldCentricReference;
    private double poseUpdateTime = Double.NEGATIVE_INFINITY;
    private boolean fieldPoseConfirmed;

    public Drive(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
    }

    @Override
    public void periodic() {
        follower.update();
        poseUpdateTime = Turret.now();
    }

    // ---------------- TeleOp ----------------

    public void manualRobotCentric(double forward, double lateral, double turn) {
        follower.manual(scaled(forward, lateral, turn));
    }

    public void manualFieldCentric(double forward, double lateral, double turn) {
        double s = org.firstinspires.ftc.teamcode.Constants.Drive.MAX_POWER;
        follower.manual(ManualDrive.fieldCentric(
                forward * s, lateral * s, turn * s, follower.pose().heading() - fieldCentricReference));
    }

    /** Makes the robot's current facing the new field-centric "forward". */
    public void resetHeading() {
        fieldCentricReference = follower.pose().heading();
    }

    public void stop() {
        follower.stop();
        follower.drivetrain.stop(); // Zero hardware immediately, even after the last scheduler tick.
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
        fieldPoseConfirmed = false;
    }

    /** Call after setting a physically verified absolute field pose, including in Auto. */
    public void confirmFieldPose(boolean confirmed) { fieldPoseConfirmed = confirmed; }
    public boolean isFieldPoseConfirmed() { return fieldPoseConfirmed; }
    public double getPoseUpdateTime() { return poseUpdateTime; }
    public boolean isLocalizationReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
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
