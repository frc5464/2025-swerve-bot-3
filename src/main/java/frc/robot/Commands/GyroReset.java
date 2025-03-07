package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class GyroReset extends Command{
    private final Runnable reset;
    

    public GyroReset(SwerveSubsystem swerveSubsystem) {
        reset = () -> {
            swerveSubsystem.zeroGyro();
        };
    }

    @Override
    public void initialize() {
        reset.run();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
