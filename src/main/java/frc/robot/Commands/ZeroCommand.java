package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ZeroCommand extends Command{
    private ElevatorSubsystem elevatorsubs;
    private WristSubsystem wristsubs;
    private ClimbSubsystem climbsubs;


    public ZeroCommand(WristSubsystem wrist, ClimbSubsystem climb, ElevatorSubsystem elevator){
        wristsubs = wrist;
        climbsubs = climb;
        elevatorsubs = elevator;

    }

    @Override
    public void initialize() {
        System.out.println("A medium Hi-C");
        wristsubs.reBoot();
        climbsubs.reBoot();
        elevatorsubs.reBoot();
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

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}
