package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ProcessorArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ProcessorInt_OutCommand extends Command{
    private final ProcessorArmSubsystem procArm;
    private boolean m_procInt_Out;

    public ProcessorInt_OutCommand(ProcessorArmSubsystem procArm, boolean m_procInt_Out){
        this.procArm = procArm;
        this.m_procInt_Out = m_procInt_Out;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(m_procInt_Out == true){
            procArm.intake(0);
        } else{
            procArm.outake(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        procArm.stopInt_Out();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
