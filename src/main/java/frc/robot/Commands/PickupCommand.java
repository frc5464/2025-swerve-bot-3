package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class PickupCommand extends Command{
    private final ElevatorSubsystem elevator;
    private final WristSubsystem wrist;
    private boolean m_pickupint;

    public PickupCommand(ElevatorSubsystem elevator, WristSubsystem intake, boolean m_pickupint) {
        this.elevator = elevator;
        this.wrist = intake;
    }

    @Override
    public void initialize() {
        elevator.level = 0.0;
        wrist.wristPickup();
    }

    @Override
    public void execute() {
        if(m_pickupint == true){
            wrist.intake(0.6);
        }
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
