package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ProcessorArmSubsystem;

public class ProcessorRotCommand extends Command{
    private final ProcessorArmSubsystem procArm;
    private boolean m_procRot;

    public ProcessorRotCommand(ProcessorArmSubsystem procArm, boolean m_procRot){
        this.procArm = procArm;
        this.m_procRot = m_procRot;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(m_procRot == true){
            procArm.rot_procarm();
        } else{
            procArm.downrot_procarm();
        }
    }

    @Override
    public void end(boolean interrupted) {
        procArm.stoprot_procarm();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}