package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ProcessorArmSubsystem {
  
  SparkMax processorRotater = new SparkMax(3, MotorType.kBrushless);
  SparkFlex processorint_out = new SparkFlex(4, MotorType.kBrushless);
  SparkMaxConfig sparkMaxConfig2 = new SparkMaxConfig();
  SparkClosedLoopController procRotPID = processorRotater.getClosedLoopController();
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0;
  double extMaxOutput = 0.2;
  double extMinOutput = 0.2;
  RelativeEncoder procrotEncoder;
  public double procrotEncoderPos;
  public double counts;
  
  public void init(){

      procrotEncoder = processorRotater.getEncoder();

    // sparkMaxConfig.closedLoop
    //   .p(kP)
    //   .i(kI)
    //   .d(kD)
    //   .outputRange(extMinOutput, extMaxOutput);
  
    // processorRotater.configure(sparkMaxConfig, null, null); 
        
  }

  public void periodic(){
  procrotEncoderPos = procrotEncoder.getPosition();
  SmartDashboard.putNumber("ProcRotEncoder", procrotEncoderPos);
  procArmToLevel(0);
  }
  
  // Roll the intake/vomit
  public void intake(double axis2){
    processorint_out.set(axis2);
  }
  public void outake(double axis3){
    processorint_out.set(-axis3);
  }
  public void stoprot(){
    processorint_out.set(0);
  }

  // Rotate the processor arm (whole thing/up and down)
  public void rot_procarm(){
    processorRotater.set(0.3);
  }
  public void downrot_procarm(){
    processorRotater.set(-0.1);
  }
  public void stoprot_procarm(){
    processorRotater.set(0);
  }

  public void procArmToLevel(int level){
    if(level == 1){
      counts = 1;
    }
    
    if(level == 2){
      counts = 2;
    }
    
    if(level == 3){
      counts = 3;
    }

    if(level == 4){
      counts  = 4;
    }

    procRotPID.setReference(counts, ControlType.kPosition);
  }
}


