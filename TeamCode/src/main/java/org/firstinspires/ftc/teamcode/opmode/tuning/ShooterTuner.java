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
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
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
    private Shooter shooter;
    private Feeder feeder;
    private boolean spinning = false;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25); // smoother graph

        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);

        GamepadEx pad = new GamepadEx(gamepad1);

        pad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> spinning = !spinning);

        pad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(()-> {feeder.setPower(1);}));

        pad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(()-> {feeder.setPower(0);}));



        schedule(new RunCommand(() -> {
            shooter.setTargetRpm(spinning ? Constants.Shooter.SHOOT_RPM : 0);
            shooter.addTelemetry(telemetry);
            telemetry.addData("Feeder RPM", feeder.getRpm());
            telemetry.update();
        }));
    }
}
