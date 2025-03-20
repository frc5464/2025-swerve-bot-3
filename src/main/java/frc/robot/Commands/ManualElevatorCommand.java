package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorCommand extends Command {
    private ElevatorSubsystem elevator;

    private boolean moving_up;

    public ManualElevatorCommand(ElevatorSubsystem elevator, boolean moving_up){
        this.elevator = elevator;
        this.moving_up = moving_up;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(Universals.manualMode == true){
            if(moving_up){
                elevator.goElevate();
            }
            else {
                elevator.reverseElevate();
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        elevator.stopElevate();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}
