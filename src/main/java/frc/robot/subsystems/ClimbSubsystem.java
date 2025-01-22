package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbSubsystem {
<<<<<<< Updated upstream
  
  SparkMax climb1 = new SparkMax(0, MotorType.kBrushless);
  SparkMax climb2 = new SparkMax(0, MotorType.kBrushless);
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
  
    climb1.configure(sparkMaxConfig, null, null);    
  }
=======
    
    SparkMax climb1 = new SparkMax(0, MotorType.kBrushless);
    SparkMax climb2 = new SparkMax(0, MotorType.kBrushless);
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    SparkClosedLoopController loopController = climb1.getClosedLoopController();
    double kP = 800000;
    double kI = 69420.1;
    double kD = 8008;
    double kIz = 1337;
    double kFF = 666;
    double extMaxOutput = -2;
    double extMinOutput = 2;

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
>>>>>>> Stashed changes
}
