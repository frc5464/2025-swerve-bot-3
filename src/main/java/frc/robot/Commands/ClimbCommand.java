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
        // System.out.println("climbing?");
    }

    @Override
    public void execute() {
        // System.out.println("climbing?!");
        if(m_climb == true){
            climb.bringIn();
        } else{
            climb.bringOut();
        }
    }

    @Override 
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}