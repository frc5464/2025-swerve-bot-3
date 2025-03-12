package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ToLevelCommand extends Command{
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final WristSubsystem chipotleWristSubsystem;

    private double m_level;
    private double chickenMcNugget;

    public ToLevelCommand(ElevatorSubsystem elevator, double level, WristSubsystem wrist, double score){
        this.m_elevatorSubsystem = elevator;
        this.chipotleWristSubsystem = wrist;
        this.m_level = level;
        this.chickenMcNugget = score;
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.level = m_level;
        chipotleWristSubsystem.targetPosition = chickenMcNugget;
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
