package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.drive.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.nectarTurret.SetNectarTurretStateCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.SetShooterRPMCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.StopShooterCommand;
import org.firstinspires.ftc.teamcode.subsystems.NectarTurret;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends CommandOpMode {

    private Robot robot;
    private GamepadEx driver;


    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, telemetry);
        driver = new GamepadEx(gamepad1);
        configureBindings();



    }

    public void configureBindings(){
        robot.drive.setDefaultCommand(
                new DefaultDriveCommand(
                        robot.drive,
                        () -> -driver.getLeftY(),   // forward  (+ = front)
                        () ->  driver.getLeftX(),   // lateral  (+ = left)
                        () -> -driver.getRightX(),  // turn     (+ = counter-clockwise)
                        true                        // field-centric; false = robot-centric
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(robot.drive::resetHeading);
        //driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new SetNectarTurretStateCommand(robot.nectarTurret, NectarTurret.State.SCORE));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new SetShooterRPMCommand(
                                robot.shooter,
                                4500,
                                150
                        )
                );

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        new StopShooterCommand(robot.shooter)
                );

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new InstantCommand(()->{robot.shooter.setFeederPower(1);})

                );

        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(
                        new InstantCommand(()->{robot.shooter.setFeederPower(0);})
                );
    }
}
