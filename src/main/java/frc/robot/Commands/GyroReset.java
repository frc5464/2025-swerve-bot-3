package frc.robot.Commands;

import frc.robot.subsystems.SwerveSubsystem;

public class GyroReset {
    private final Runnable reset;

    public GyroReset(SwerveSubsystem swerveSubsystem) {
        reset = () -> {
            swerveSubsystem.zeroGyro();
        };
    }

    // @Override
    public void initialize() {
        reset.run();
    }

    // @Override
    public boolean isFinished() {
        return true;
    }
}
