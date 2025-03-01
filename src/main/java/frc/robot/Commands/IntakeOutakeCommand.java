package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class IntakeOutakeCommand extends Command {
    private final WristSubsystem m_wristSubsystem;
    private boolean m_intake;

    public IntakeOutakeCommand(WristSubsystem a, boolean intake){
        this.m_wristSubsystem = a;
        this.m_intake = intake;
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
