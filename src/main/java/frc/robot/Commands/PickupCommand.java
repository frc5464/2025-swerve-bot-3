package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class PickupCommand extends Command{
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final WristSubsystem m_wristSubsystem;

    public PickupCommand(ElevatorSubsystem level, WristSubsystem intake) {
        this.m_elevatorSubsystem = level;
        this.m_wristSubsystem = intake;
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.level = 0.0;
        m_wristSubsystem.wristPickup();
    }

    @Override
    public void execute() {
        m_wristSubsystem.intake(0.6);
    }

    @Override
    public void end(boolean interrupted) {
        m_wristSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
