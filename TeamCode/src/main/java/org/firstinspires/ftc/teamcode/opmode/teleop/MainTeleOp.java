package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.gamepad.*;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.aiming.*;
import org.firstinspires.ftc.teamcode.commands.drive.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.*;
import org.firstinspires.ftc.teamcode.commands.turret.HomeTurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name="Main TeleOp",group="Competition")
public class MainTeleOp extends CommandOpMode {
    private Robot robot;
    private Intake intake;
    private TrackTargetCommand largeTrack,smallTrack;
    private Command largeHome,smallHome;
    @Override public void initialize() {
        robot=new Robot(hardwareMap,true);
        intake=new Intake(hardwareMap);
        robot.drive.setPose(new Pose(ShootingConfig.START_X,ShootingConfig.START_Y,
                Math.toRadians(ShootingConfig.START_HEADING_DEG)));
        robot.drive.confirmFieldPose(ShootingConfig.START_POSE_CONFIRMED);
        largeTrack=new TrackTargetCommand(robot.drive,robot.large,robot.targets);
        smallTrack=new TrackTargetCommand(robot.drive,robot.small,robot.targets);
        largeHome=new HomeTurretCommand(robot.large.turret);
        smallHome=new HomeTurretCommand(robot.small.turret);
        GamepadEx driver=new GamepadEx(gamepad1), operator=new GamepadEx(gamepad2);
        robot.drive.setDefaultCommand(new DefaultDriveCommand(robot.drive,
                driver::getLeftY,()->-driver.getLeftX(),()->-driver.getRightX(),true));
        // GamepadEx already inverts stick Y; Pedro lateral-positive is LEFT.
        // Field-centric reference only: never change the global field pose for a driver reset.
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(robot.drive::resetHeading);
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(intake::toggle, intake));
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(largeHome);
        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(smallHome);
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(largeTrack);
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(smallTrack);
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ShootOneCommand(robot.large,largeTrack));
        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ShootOneCommand(robot.small,smallTrack));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(()->select(FieldTargets.Alliance.RED,robot.targets.getEnd()));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(()->select(FieldTargets.Alliance.BLUE,robot.targets.getEnd()));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()->select(robot.targets.getAlliance(),FieldTargets.End.AUDIENCE));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()->select(robot.targets.getAlliance(),FieldTargets.End.SCORING));
        operator.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(()-> {
            stopMechanisms(); intake.stop();
        });
        schedule(new RunCommand(()-> {
            telemetry.addData("Alliance/end",robot.targets.getAlliance()+" / "+robot.targets.getEnd());
            telemetry.addData("Pose",robot.drive.getPose());
            telemetry.addData("Intake",intake.isRunning() ? "ON" : "OFF");
            report(robot.large,largeTrack); report(robot.small,smallTrack); telemetry.update();
        }));
    }
    private void select(FieldTargets.Alliance alliance,FieldTargets.End end) {
        // Clear any queued feed request and require explicit re-arm after target change.
        stopMechanisms(); robot.targets.select(alliance,end);
    }
    private void stopMechanisms() {
        CommandScheduler.getInstance().cancel(largeTrack,smallTrack,largeHome,smallHome);
        // Cancel feeder commands through their owners, without cancelling drive/telemetry.
        Command a=robot.large.feeder.getCurrentCommand(), b=robot.small.feeder.getCurrentCommand();
        if(a!=null) a.cancel(); if(b!=null) b.cancel();
        robot.large.stop(); robot.small.stop();
    }
    private void report(ShotChannel ch,TrackTargetCommand track) {
        telemetry.addData(ch.config.name+" aim",track.status());
        telemetry.addData(ch.config.name+" range in",track.distanceInches());
        telemetry.addData(ch.config.name+" turret",ch.turret.state()+" / "+ch.turret.angleDegrees());
        telemetry.addData(ch.config.name+" fault",ch.turret.fault());
        telemetry.addData(ch.config.name+" hood deg",ch.hood.targetDegrees());
        telemetry.addData(ch.config.name+" feeder configured",ch.feeder.available());
        ch.shooter.addTelemetry(telemetry);
    }
    @Override public void reset() {
        if(intake!=null) intake.stop();
        if(robot!=null) robot.stop();
        super.reset();
    }
}

