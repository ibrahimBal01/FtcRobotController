package org.firstinspires.ftc.teamcode.commands.turret;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.aiming.TurretController;
public class SetTurretAngleCommand extends CommandBase {
    private final Turret turret; private final double angle; private boolean reachable;
    public SetTurretAngleCommand(Turret turret,double angle) { this.turret=turret; this.angle=angle; addRequirements(turret); }
    @Override public void initialize() { reachable=turret.setTargetDegrees(angle); }
    @Override public boolean isFinished() { return !reachable || turret.atTarget() || turret.state()==TurretController.State.FAULT; }
    @Override public void end(boolean interrupted) { turret.stop(); }
}
