package frc.robot.subsystems;

import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import java.io.File;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class SwerveSubsystem extends SubsystemBase{
    private SwerveDrive m_robotDrive;

    public SwerveSubsystem(){
        try {
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
            m_robotDrive=new SwerveParser(
                new File(Filesystem.getDeployDirectory(),"swerve"
                )).createSwerveDrive(Units.feetToMeters(14.5));
    
            } catch (Exception e) {
            throw new RuntimeException(e);
            }
    }

    public void periodic(){
        SmartDashboard.putNumber("Yaw", m_robotDrive.getYaw().getDegrees());
        SmartDashboard.putNumber("IMU angle", m_robotDrive.getGyro().getRawRotation3d().getAngle());
        
         swervelib.SwerveModule[] modules = m_robotDrive.getModules();
         SmartDashboard.putNumber("FL Enc", modules[0].getAbsolutePosition());
         SmartDashboard.putNumber("FR Enc", modules[1].getAbsolutePosition());
         SmartDashboard.putNumber("BL Enc", modules[2].getAbsolutePosition());
         SmartDashboard.putNumber("BR Enc", modules[3].getAbsolutePosition());

    }

    public void drive(double x, double y, double z){
        m_robotDrive.drive(new Translation2d(x * 1.5,y * 1.5),z * 3, true, false);
    }

    public void zeroGyro(){
        m_robotDrive.zeroGyro();
    }
}
