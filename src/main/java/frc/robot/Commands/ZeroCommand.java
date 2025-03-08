package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class ZeroCommand extends Command{
    
        private WristSubsystem wristsubs;
    public ZeroCommand(WristSubsystem wrist){
        this.wristsubs = wrist;


    }

    @Override
    public void initialize() {
        wristsubs.reBoot();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
