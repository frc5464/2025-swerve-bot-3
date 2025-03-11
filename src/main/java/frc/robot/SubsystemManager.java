package frc.robot;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ProcessorArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class SubsystemManager {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    // private final ProcessorArmSubsystem processorArmSubsystem = new ProcessorArmSubsystem();
    // private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    public SubsystemManager() {
        // swerveSubsystem.();
    }

    // public VisionSubsystem getVisionSubsystem() {
    //     return visionSubsystem;
    // }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }
    
    public WristSubsystem getWristSubsystem() {
        return wristSubsystem;
    }

    public ElevatorSubsystem getElevatorSubsystem() {
        return elevatorSubsystem;
    }

    public ClimbSubsystem getClimbSubsystem() {
        return climbSubsystem;
    }

    // public ProcessorArmSubsystem getProcessorArmSubsystem() {
    //     return processorArmSubsystem;
    // }
}
