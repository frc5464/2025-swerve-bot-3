package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ClimbCommand extends Command{
    private final ClimbSubsystem climb;
    private boolean m_climb;

    public ClimbCommand(ClimbSubsystem climb, boolean m_climb){
        this.climb = climb;
        this.m_climb = m_climb;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(m_climb == true){
            climb.bringIn();
        } else{
            climb.bringOut();
        }
    }

    @Override 
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
