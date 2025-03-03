package frc.robot.Commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;
import frc.robot.subsystems.WristSubsystem;

public class IntakeOutakeCommand extends Command {
    private final WristSubsystem wrist;
    private boolean m_intake;

    public IntakeOutakeCommand(WristSubsystem wrist, boolean m_intake){
        this.wrist = wrist;
        this.m_intake = m_intake;
    }

    @Override
    public void initialize() {
        Universals.coralIntaking = true;
        if(RobotState.isAutonomous()){
            Universals.homingPathtoCoral = true;
        }
    }

    @Override
    public void execute() {
        if(Universals.coralIntaking){
            wrist.Intake();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
