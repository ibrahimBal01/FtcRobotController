package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.SetHoodPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Flywheel tuning with FTC Dashboard (http://192.168.43.1:8080/dash on a Control Hub).
 *
 * - Graph tab: plot "Shooter target RPM" and "Shooter RPM" (add raw RPM / power if you like).
 * - Configuration tab, group "Shooter": edit SHOOT_RPM, kV, kP, kI, kD, ... live and press save.
 *   SHOOT_RPM is re-applied every loop here, so changing it steps the target.
 * - Gamepad 1: A = toggle spin, hold RIGHT_BUMPER = feed (only while at speed).
 * Only the Shooter and Feeder are created, so no drivetrain is needed.
 */
@TeleOp(name = "Shooter Tuner", group = "Tuning")
public class ShooterTuner extends CommandOpMode {
    private Shooter large_shooter;
    private Shooter small_shooter;
    private Hood small_hood;
    private Hood large_hood;
    private Feeder feeder;
    private boolean spinning = false;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25); // smoother graph

        large_shooter = new Shooter(hardwareMap, "Large", Constants.Shooters.LARGE);
        small_shooter = new Shooter(hardwareMap, "Small", Constants.Shooters.SMALL);

        small_hood = new Hood(hardwareMap, "Small", Constants.Shooters.SMALL);
        large_hood = new Hood(hardwareMap, "Large", Constants.Shooters.LARGE);


        feeder = new Feeder(hardwareMap);

        GamepadEx pad = new GamepadEx(gamepad1);

        pad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> spinning = !spinning);

        pad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new FeedCommand(feeder, large_shooter, small_shooter));

        pad.getGamepadButton(GamepadKeys.Button.A)
                        .whenPressed(new SetHoodPositionCommand(small_hood,0.0));

        pad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SetHoodPositionCommand(small_hood,1.0));

        pad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SetHoodPositionCommand(large_hood,0.0));

        pad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SetHoodPositionCommand(large_hood,1.0));




        schedule(new RunCommand(() -> {
            large_shooter.setTargetRpm(spinning ? Constants.Shooters.LARGE.shootRpm : 0);
            large_shooter.addTelemetry(telemetry);

            small_shooter.setTargetRpm(spinning ? Constants.Shooters.SMALL.shootRpm : 0);
            small_shooter.addTelemetry(telemetry);

            small_hood.addTelemetry(telemetry);
            large_hood.addTelemetry(telemetry);
            

            telemetry.update();
        }));
    }
}
