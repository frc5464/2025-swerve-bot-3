package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitCommand extends Command{
    double m_time;
    Timer timer;
    public WaitCommand(double time){
        m_time = time;
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        if(timer.get() > m_time){
            return true;
        }
        else{
            return false;
        }
    }    

}
