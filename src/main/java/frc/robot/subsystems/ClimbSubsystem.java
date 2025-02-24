package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSubsystem {
  
  SparkMax climb = new SparkMax(2, MotorType.kBrushless);
  SparkMaxConfig sparkMaxConfig3 = new SparkMaxConfig();
  // SparkClosedLoopController loopController = climb1.getClosedLoopController();
  SparkClosedLoopController climbPID = climb.getClosedLoopController();
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0;
  double extMaxOutput = 0;
  double extMinOutput = 0;
  RelativeEncoder climbEncoder;
  public double climbEncoderPos;
  public double counts;
  

  public void init(){

    climbEncoder = climb.getEncoder();

                                              

    // sparkMaxConfig.closedLoop
    //   .p(kP)
    //   .i(kI)
    //   .d(kD)
    //   .outputRange(extMinOutput, extMaxOutput);
  
    //climb1.configure(sparkMaxConfig, null, null);    
  }

    public void periodic(){
    climbEncoderPos = climbEncoder.getPosition();
    SmartDashboard.putNumber("ClimbEncoder", climbEncoderPos);
    //ClimbToLevel(0);
        //loopController.setReference(400, ControlType.kPosition );
        
    } 
    
    public void bringOut(){

      climb.set(1);
    }
    public void bringIn(){
      climb.set(-1);
    }
    public void stop(){
      climb.set(0);
    }

    public void reBoot(){
      climbEncoder.setPosition(0);
    }

    // public void ClimbToLevel(int level){
    //   if(level == 1){
    //     counts = 1;
    //   }
      
    //   if(level == 2){
    //     counts = 2;
    //   }

    //   climbPID.setReference(counts, ControlType.kPosition);
    // }
}

