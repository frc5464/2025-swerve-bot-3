package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.SubsystemManager;
import frc.robot.Commands.IntakeOutakeCommand;
import frc.robot.Commands.ManualElevatorCommand;
import frc.robot.Commands.BrakeCommand;
import frc.robot.Commands.ClimbCommand;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.GyroReset;
import frc.robot.Commands.ManualModeCommand;
import frc.robot.Commands.PickupCommand;
import frc.robot.Commands.ProcessorInt_OutCommand;
import frc.robot.Commands.ProcessorRotCommand;
import frc.robot.Commands.SlowModeCommand;
import frc.robot.Commands.ToLevelCommand;
import frc.robot.Commands.ZeroCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ProcessorArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class OperatorInterface {
    private static final CommandJoystick driver = new CommandJoystick(0);
    private static final CommandJoystick mineController = new CommandJoystick(1);

    /**
     * Connects this to joysticks
     * 
     * 
     * @param subsystemManager
     */
    public static void create(SubsystemManager subsystemManager){
        final SwerveSubsystem drive = subsystemManager.getSwerveSubsystem();
        final WristSubsystem wrist = subsystemManager.getWristSubsystem();
        final ElevatorSubsystem elevator = subsystemManager.getElevatorSubsystem();
        final ClimbSubsystem climb = subsystemManager.getClimbSubsystem();
        // final ProcessorArmSubsystem processor = subsystemManager.getProcessorArmSubsystem();
        
        //Drive Controller
        driver.axisGreaterThan(2, 0.1).whileTrue(new IntakeOutakeCommand(wrist, true));
        driver.axisGreaterThan(2, 0.1).whileTrue(new PickupCommand(elevator, wrist));
        driver.button(5).whileTrue(new IntakeOutakeCommand(wrist, true));
        driver.axisGreaterThan(3, 0).whileTrue(new IntakeOutakeCommand(wrist, false));
        driver.button(1).onTrue(new ToLevelCommand(elevator, 1, wrist, 16));
        driver.button(2).onTrue(new ToLevelCommand(elevator, 2, wrist, 16));
        driver.button(3).onTrue(new ToLevelCommand(elevator, 3, wrist, 16));
        driver.button(4).onTrue(new ToLevelCommand(elevator, 4, wrist, 17.5));
        
        driver.button(7).onTrue(new GyroReset(drive));
        driver.button(8).onTrue(new ZeroCommand(wrist, climb, elevator));
        
        drive.setDefaultCommand(new DriveCommand(drive, driver)); 

        // mineController.axisGreaterThan(2, 0.1).whileTrue(new ProcessorInt_OutCommand(processor, true));
        // mineController.axisGreaterThan(3, 0.1).whileTrue(new ProcessorRotCommand(processor, true));
        mineController.axisGreaterThan(5, 0.1).whileTrue(new ClimbCommand(climb, false));
        mineController.axisLessThan(5, -0.1).whileTrue(new ClimbCommand(climb, true));
        mineController.button(5).whileTrue(new ManualModeCommand());
        mineController.pov(0).whileTrue(new ManualElevatorCommand(elevator, true));
        mineController.pov(180).whileTrue(new ManualElevatorCommand(elevator, false));
        mineController.button(1).whileTrue(new BrakeCommand(drive));
        mineController.button(2).whileTrue(new SlowModeCommand());
    }

        private OperatorInterface(){
    }
}
