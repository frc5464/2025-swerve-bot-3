package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ProcessorArmSubsystem {
  SparkFlex processorRotater = new SparkFlex(0, MotorType.kBrushless);
  SparkFlex processorint_out = new SparkFlex(0, MotorType.kBrushless);
  SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0;
  double extMaxOutput = 0;
  double extMinOutput = 0;
  
  public void init(){
    sparkMaxConfig.closedLoop
      .p(kP)
      .i(kI)
      .d(kD)
      .outputRange(extMinOutput, extMaxOutput);
  
    processorRotater.configure(sparkMaxConfig, null, null); 
        
  }
  // Roll the intake/vomit
  public void roll_procarm(){
    processorint_out.set(0);
  }
  public void stoproll_procarm(){
    processorint_out.set(0);
  }
  public void revroll_procarm(){
    processorint_out.set(-0);
  }
  // Rotate the processor arm (whole thing/up and down)
  public void rot_procarm(){
    processorRotater.set(0);
  }
  public void stoprot_procarm(){
    processorRotater.set(0);
  }
  public void downrot_procarm(){
    processorRotater.set(-0);
  }
}
