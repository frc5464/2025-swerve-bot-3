package frc.robot.Commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;
import frc.robot.subsystems.WristSubsystem;

public class IntakeOutakeCommand extends Command {
    private final WristSubsystem wristsubs;
    private boolean m_intake;

    private Timer timer = new Timer();

    public IntakeOutakeCommand(WristSubsystem wrist, boolean m_intake){
        this.wristsubs = wrist;
        this.m_intake = m_intake;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
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

        // This should cause autonomous to only spit out game pieces for a half second
        if((timer.get() > 0.5) && RobotState.isAutonomous() && (!m_intake)){
            return true;
        }
        return false;
    }
}
