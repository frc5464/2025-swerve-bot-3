package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.SubsystemManager;
import frc.robot.Commands.PickupCommand;
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
        final ProcessorArmSubsystem processor = subsystemManager.getProcessorArmSubsystem();

        //Drive Controller
        driver.axisGreaterThan(2, 0.1).whileTrue(new PickupCommand(elevator, wrist));
    }

        private OperatorInterface(){
    }
}
