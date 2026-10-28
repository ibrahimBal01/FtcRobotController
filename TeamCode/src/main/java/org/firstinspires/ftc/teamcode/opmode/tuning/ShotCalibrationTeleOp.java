package org.firstinspires.ftc.teamcode.opmode.tuning;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.gamepad.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.ShootingConfig;
import org.firstinspires.ftc.teamcode.aiming.ShotChannel;
import org.firstinspires.ftc.teamcode.commands.turret.HomeTurretCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedCommand;

/** Stationary, single-channel bench calibration. No drive or automatic field aiming. */
@Config
@TeleOp(name="Shot Calibration (stationary)",group="Tuning")
public class ShotCalibrationTeleOp extends CommandOpMode {
    public static boolean LARGE_CHANNEL=false; // Select before INIT.
    public static double RPM=2500, HOOD_DEG=35, TURRET_DEG=0;
    private ShotChannel ch;
    private Command arm,home,feed;
    private boolean armed,requestValid;
    @Override public void initialize() {
        telemetry=new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
        ch=new ShotChannel(hardwareMap,LARGE_CHANNEL?ShootingConfig.LARGE:ShootingConfig.SMALL);
        home=new HomeTurretCommand(ch.turret);
        arm=new FunctionalCommand(
                ()->{armed=true;requestValid=false;},
                ()->{
                    requestValid=ch.turret.setTargetDegrees(TURRET_DEG) && ch.hood.setAngle(HOOD_DEG)
                            && Double.isFinite(RPM) && RPM>0 && RPM<=ch.config.flywheel.maxRpm;
                    ch.shooter.setTargetRpm(requestValid?RPM:0);
                },
                interrupted->{armed=false;requestValid=false;ch.stop();},
                ()->false,ch.turret,ch.hood,ch.shooter);
        feed=new FeedCommand(ch.feeder,ch.shooter,
                ()->armed && requestValid && ch.turret.atTarget() && ch.hood.atTarget());
        GamepadEx p=new GamepadEx(gamepad1);
        p.getGamepadButton(GamepadKeys.Button.A).whenPressed(home);
        p.getGamepadButton(GamepadKeys.Button.X).whenPressed(arm);
        p.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(feed);
        p.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(()->{
            arm.cancel();home.cancel();feed.cancel();armed=false;requestValid=false;ch.stop();
        });
        schedule(new RunCommand(()->{
            telemetry.addData("Channel",ch.config.name);
            telemetry.addData("Controls","A home; X arm; RB one shot; BACK stop");
            telemetry.addData("Turret state",ch.turret.state());
            telemetry.addData("Turret degrees",ch.turret.angleDegrees());
            telemetry.addData("Home switch",ch.turret.homeSwitchActive());
            telemetry.addData("Fault",ch.turret.fault());
            telemetry.addData("Hood commanded degrees",ch.hood.targetDegrees());
            telemetry.addData("Hood settled (estimated)",ch.hood.atTarget());
            telemetry.addData("Feeder available",ch.feeder.available());
            telemetry.addData("Armed / valid",armed+" / "+requestValid);
            ch.shooter.addTelemetry(telemetry);telemetry.update();
        }));
    }
    @Override public void reset() { if(ch!=null)ch.stop();super.reset(); }
}

