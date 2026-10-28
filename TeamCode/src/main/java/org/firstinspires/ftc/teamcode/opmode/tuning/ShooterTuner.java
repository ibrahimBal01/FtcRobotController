package org.firstinspires.ftc.teamcode.opmode.tuning;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.gamepad.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.ShootingConfig;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/** Independent flywheel bench tuner. No turret, hood, feeder or drive required. */
@TeleOp(name="Shooter Tuner",group="Tuning")
public class ShooterTuner extends CommandOpMode {
    private Shooter large,small; private boolean largeOn,smallOn;
    @Override public void initialize() {
        telemetry=new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
        large=new Shooter(hardwareMap,ShootingConfig.LARGE.flywheel);
        small=new Shooter(hardwareMap,ShootingConfig.SMALL.flywheel);
        GamepadEx p=new GamepadEx(gamepad1);
        p.getGamepadButton(GamepadKeys.Button.A).whenPressed(()->largeOn=!largeOn);
        p.getGamepadButton(GamepadKeys.Button.B).whenPressed(()->smallOn=!smallOn);
        p.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(()->{largeOn=false;smallOn=false;large.stop();small.stop();});
        schedule(new RunCommand(()-> {
            large.setTargetRpm(largeOn?ShootingConfig.LARGE.flywheel.tuneRpm:0);
            small.setTargetRpm(smallOn?ShootingConfig.SMALL.flywheel.tuneRpm:0);
            large.addTelemetry(telemetry); small.addTelemetry(telemetry); telemetry.update();
        }));
    }
    @Override public void reset() { if(large!=null)large.stop(); if(small!=null)small.stop(); super.reset(); }
}

