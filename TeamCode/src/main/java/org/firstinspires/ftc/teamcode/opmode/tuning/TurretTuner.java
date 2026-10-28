package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.shooter.SetTurretAngleCommand;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Turret Tuner", group = "Tuning")
public class TurretTuner extends CommandOpMode {
    private Turret large_turret;
    private Turret small_turret;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25); // smoother graph

        large_turret = new Turret(hardwareMap, "LargeTurret", Constants.Turrets.LARGE);
        small_turret = new Turret(hardwareMap, "SmallTurret", Constants.Turrets.SMALL);

        GamepadEx pad = new GamepadEx(gamepad1);

        pad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SetTurretAngleCommand(large_turret, 50));
        pad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SetTurretAngleCommand(large_turret, -50));

        pad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SetTurretAngleCommand(small_turret, 50));
        pad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SetTurretAngleCommand(small_turret, -50));


        schedule(new RunCommand(() -> {
            large_turret.addTelemetry(telemetry);
            small_turret.addTelemetry(telemetry);
            telemetry.update();
        }));

    }
}
