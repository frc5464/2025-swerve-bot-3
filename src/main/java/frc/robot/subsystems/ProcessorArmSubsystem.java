package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ProcessorArmSubsystem {
  
  SparkMax processorRotater = new SparkMax(35, MotorType.kBrushless);
  SparkFlex processorint_out = new SparkFlex(10, MotorType.kBrushless);
  SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  SparkClosedLoopController procRotPid = processorRotater.getClosedLoopController();
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0;
  double extMaxOutput = 0.2;
  double extMinOutput = 0.2;
  RelativeEncoder procrotEncoder;
  public double procrotEncoderPos;
  
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
  }

 

  
  
  // Roll the intake/vomit
  public void roll_procarm(){
    processorint_out.set(0.1);
  }
  public void revroll_procarm(){
    processorint_out.set(-0.1);
  }
  public void stoproll_procarm(){
    processorint_out.set(0);
  }

  // Rotate the processor arm (whole thing/up and down)
  public void rot_procarm(){
    processorRotater.set(0.1);
  }
  public void downrot_procarm(){
    processorRotater.set(-0.1);
  }
  public void stoprot_procarm(){
    processorRotater.set(0);
  }
}


