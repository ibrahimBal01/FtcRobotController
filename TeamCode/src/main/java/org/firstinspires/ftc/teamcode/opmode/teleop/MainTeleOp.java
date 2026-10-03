package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
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
                        () -> -driver.getLeftY(),   // forward  (+ = front)
                        () ->  driver.getLeftX(),   // lateral  (+ = left)
                        () -> -driver.getRightX(),  // turn     (+ = counter-clockwise)
                        true                        // field-centric; false = robot-centric
                )
        );

        //driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(robot.drive::resetHeading);
        //driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new SetNectarTurretStateCommand(robot.nectarTurret, NectarTurret.State.SCORE));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(robot.drive::resetHeading);
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new SetShooterRpmCommand(robot.shooter, Constants.Shooter.SHOOT_RPM));
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(new StopShooterCommand(robot.shooter));
        // Shooter: RIGHT_BUMPER = spin up, B = stop, hold A = feed (only while at speed)
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new FeedCommand(robot.feeder, robot.shooter));

        // Telemetry for tuning the flywheel
        schedule(new RunCommand(() -> {
            telemetry.addData("Shooter target RPM", robot.shooter.getTargetRpm());
            telemetry.addData("Shooter RPM", robot.shooter.getRpm());
            telemetry.addData("At speed", robot.shooter.atSpeed());
            telemetry.addData("Robot Pose", robot.drive.getPose());
            telemetry.addData("Feeder RPM", robot.feeder.getRpm());
            telemetry.update();
        }));
    }
}
