package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ToLevelCommand extends Command{
    private final ElevatorSubsystem m_elevatorSubsystem;
    private double m_level;

    public ToLevelCommand(ElevatorSubsystem elevator, double level){
        this.m_elevatorSubsystem = elevator;
        this.m_level = level;
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
