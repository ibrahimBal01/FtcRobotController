package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.arm.ScoreSequence;
import org.firstinspires.ftc.teamcode.commands.arm.SetArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends CommandOpMode {

    private Robot robot;
    private GamepadEx driver;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        driver = new GamepadEx(gamepad1);

        robot.drive.setDefaultCommand(
                new DefaultDriveCommand(
                        robot.drive,
                        driver::getLeftX,
                        () -> -driver.getLeftY(),
                        driver::getRightX
                )
        );

        new GamepadButton(driver, GamepadKeys.Button.A)
                .whenPressed(new SetArmStateCommand(robot.arm, Arm.State.INTAKE));

        new GamepadButton(driver, GamepadKeys.Button.B)
                .whenPressed(new SetArmStateCommand(robot.arm, Arm.State.HOME));

        new GamepadButton(driver, GamepadKeys.Button.X)
                .whenPressed(new ScoreSequence(robot.arm));
    }
}
