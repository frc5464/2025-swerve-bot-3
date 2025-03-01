package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Universals;

public class ManualModeCommand extends Command{
    @Override
    public void initialize() {
        Universals.manualMode = !Universals.manualMode; 
    }
}
