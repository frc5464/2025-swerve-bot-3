package frc.robot.subsystems;

import swervelib.parser.SwerveParser;

import java.io.File;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveDrive;

public class SwerveSubsystem {
    private SwerveDrive m_robotDrive;
    
    public SwerveSubsystem(){
        try {
        m_robotDrive=new SwerveParser(
            new File(Filesystem.getDeployDirectory(),"swerve"
            )).createSwerveDrive(
                Units.feetToMeters(14.5));

        } catch (Exception e) {
        throw new RuntimeException(e);
        }
    }

    public void periodic(){
        SmartDashboard.putNumber("Yaw", m_robotDrive.getYaw().getDegrees());
        SmartDashboard.putNumber("IMU angle", m_robotDrive.getGyro().getRawRotation3d().getAngle());
        
        // swervelib.SwerveModule[] modules = m_robotDrive.getModules();
        // SmartDashboard.putNumber("FL Enc", modules[0].getAbsolutePosition());

    }

    public void drive(double x, double y, double z){
        m_robotDrive.drive(new Translation2d(x,y),z, true, false);
    }

    public void zeroGyro(){
        m_robotDrive.zeroGyro();
    }
}
