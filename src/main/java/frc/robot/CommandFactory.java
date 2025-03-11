package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandFactory {


    public CommandFactory(SubsystemManager subsystemManager) {
        

    }


    public Command AutonomousRun(String autoName){
        SequentialCommandGroup auto = new SequentialCommandGroup();
        // auto.addCommands(new WaitCommand(Universals.wait));
        // auto.addCommands(new GyroReset(swerveSubsystem));
        auto.addCommands(new PathPlannerAuto(autoName));

        return auto;
    }
}
