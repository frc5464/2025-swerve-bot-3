package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command{
    private final ClimbSubsystem m_climbSubsystem;
    
    public ClimbCommand(ClimbSubsystem climb){
        this.m_climbSubsystem = climb;
    }

    @Override
    public void initialize() {

    }

    @Override 
    public void execute() {

    }

    @Override 
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
