package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbSubsystem {
  
  SparkMax climb1 = new SparkMax(0, MotorType.kBrushless);
  SparkMax climb2 = new SparkMax(0, MotorType.kBrushless);
  SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  SparkClosedLoopController loopController = climb1.getClosedLoopController();
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
  
    climb1.configure(sparkMaxConfig, null, null);    
  }

    public void periodic(){

        loopController.setReference(400, ControlType.kPosition );
        
    }
}
