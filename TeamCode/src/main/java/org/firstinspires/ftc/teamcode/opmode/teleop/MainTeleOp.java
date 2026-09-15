package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.drive.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.nectarTurret.SetNectarTurretStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.NectarTurret;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends CommandOpMode {

    private Robot robot;
    private GamepadEx driver;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        driver = new GamepadEx(gamepad1);
        configureBindings();



    }

    public void configureBindings(){
        robot.drive.setDefaultCommand(
                new DefaultDriveCommand(
                        robot.drive,
                        driver::getLeftX,
                        () -> -driver.getLeftY(),
                        driver::getRightX
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(robot.drive::resetHeading);
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new SetNectarTurretStateCommand(robot.nectarTurret, NectarTurret.State.SCORE));

    }
}
