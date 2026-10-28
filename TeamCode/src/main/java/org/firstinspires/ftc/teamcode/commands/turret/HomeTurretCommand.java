package org.firstinspires.ftc.teamcode.commands.turret;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.aiming.TurretController;
public class HomeTurretCommand extends CommandBase {
    private final Turret turret;
    public HomeTurretCommand(Turret turret) { this.turret=turret; addRequirements(turret); }
    @Override public void initialize() { turret.home(); }
    @Override public boolean isFinished() { return turret.state()==TurretController.State.HOLD
            || turret.state()==TurretController.State.FAULT; }
    @Override public void end(boolean interrupted) { turret.stop(); }
}
