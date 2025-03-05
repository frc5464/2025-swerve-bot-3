package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends Command{
    
    SwerveSubsystem swurv;
    Joystick shtick;

    public DriveCommand(SwerveSubsystem swerb, CommandJoystick controller){
        swurv = swerb;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        // introduce deadband to keep controller drift from causing issues
        double driveX = shtick.getRawAxis(1);
        double driveY = shtick.getRawAxis(0);
        double driveRot = -shtick.getRawAxis(4);
        if(Math.abs(driveX) < 0.1){ driveX = 0;}
        if(Math.abs(driveY) < 0.1){ driveY = 0;}
        if(Math.abs(driveRot) < 0.1){ driveRot = 0;}

        swurv.drive(driveX, driveY, driveRot);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
