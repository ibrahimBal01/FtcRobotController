package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.drive.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.SetShooterRpmCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.StopShooterCommand;

@TeleOp(name = "Shooter Test", group = "Competition")
public class ShooterTest extends CommandOpMode {

    private Robot robot;
    private GamepadEx driver;


    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        driver = new GamepadEx(gamepad1);
        configureBindings();



    }

    public void configureBindings(){


        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new SetShooterRpmCommand(robot.shooter, Constants.Shooter.SHOOT_RPM));

        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(new SetShooterRpmCommand(robot.shooter, 3500));

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SetShooterRpmCommand(robot.shooter, 4000));

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new SetShooterRpmCommand(robot.shooter, 500));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new StopShooterCommand(robot.shooter));
        // Shooter: RIGHT_BUMPER = spin up, B = stop, hold A = feed (only while at speed)
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(()->{robot.feeder.setPower(1);}));

        // Telemetry for tuning the flywheel
        schedule(new RunCommand(() -> {
            telemetry.addData("Shooter target RPM", robot.shooter.getTargetRpm());
            telemetry.addData("Shooter RPM", robot.shooter.getRpm());
            telemetry.addData("At speed", robot.shooter.atSpeed());
            telemetry.addData("Robot Pose", robot.drive.getPose());
            telemetry.update();
        }));
    }
}
