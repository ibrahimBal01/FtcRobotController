package org.firstinspires.ftc.teamcode.commands.shooter;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
public class SetHoodAngleCommand extends CommandBase {
    private final Hood hood; private final double degrees; private boolean valid;
    public SetHoodAngleCommand(Hood hood,double degrees) { this.hood=hood; this.degrees=degrees; addRequirements(hood); }
    @Override public void initialize() { valid=hood.setAngle(degrees); }
    @Override public boolean isFinished() { return !valid || hood.atTarget(); }
    @Override public void end(boolean interrupted) { if(interrupted) hood.stop(); }
}
