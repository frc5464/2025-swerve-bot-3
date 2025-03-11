package frc.robot.Commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;
import frc.robot.subsystems.WristSubsystem;

public class IntakeOutakeCommand extends Command {
    private final WristSubsystem wristsubs;
    private boolean m_intake;

    public IntakeOutakeCommand(WristSubsystem wrist, boolean m_intake){
        this.wristsubs = wrist;
        this.m_intake = m_intake;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(m_intake == true){
            wristsubs.intake(1);
        } else{
            wristsubs.outake(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        wristsubs.stop();
    }

    @Override
    public boolean isFinished() {
        // if(wrist.getIntOutCurrent() > 38){
        //     return true;
        // }
        return false;
    }
}
