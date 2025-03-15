package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class BrakeCommand extends Command{
    private SwerveSubsystem swerve;
    // private boolean brake;

    public BrakeCommand(SwerveSubsystem swerve){
        this.swerve = swerve;
        // this.brake = brake;//lkajs;dlkajsd
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        swerve.lockpose();
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
