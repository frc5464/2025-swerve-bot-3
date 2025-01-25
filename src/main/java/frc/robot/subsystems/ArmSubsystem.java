package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem {
  
  SparkMax armAlgae = new SparkMax(32, MotorType.kBrushless);
  SparkMax armCoral = new SparkMax(33, MotorType.kBrushless);
  SparkMax armRot = new SparkMax(34, MotorType.kBrushless);
  SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  SparkClosedLoopController loopController1 = armRot.getClosedLoopController();
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0;
  double extMaxOutput = 0;
  double extMinOutput = 0;
  RelativeEncoder armEncoder;
  public double encoderPos;

  public void init(){
    
    armEncoder = armRot.getEncoder();
  //Encoderstuff
  // sparkMaxConfig.closedLoop
  //   .p(kP)
  //   .i(kI)
  //   .d(kD)
  //   .outputRange(extMinOutput, extMaxOutput);
  
  // armRot.configure(sparkMaxConfig, null, null);
  
  }

  public void periodic(){
  encoderPos = armEncoder.getPosition();
  SmartDashboard.putNumber("Encoder", encoderPos);
  }

  

  //Drop Coral
  public void dropCoral(double axi2){
    armCoral.set(axi2);
  }
  public void retrieveCoral(double axi3){
    armCoral.set(-axi3);
  }
  //Kick Algea
  public void windUp(){
    armAlgae.set(0.2);
  }
  //Rotate arm
  public void rotArm(){
  armRot.set(0.3);
  }
  public void revrotArm(){
  armRot.set(-0.3);
  }
  //Stop arm movement
  public void stopArm(){
  armRot.set(0);
  }

  //Get arm to the 1st stage
  public void lvl1Arm(){

    loopController1.setReference(333, ControlType.kPosition);

  }
  
  //Get arm to the 2nd stage
  public void lvl2Arm(){

    loopController1.setReference(444, ControlType.kPosition);

  }
  
  //Get arm to the 3rd stage
  public void lvl3Arm(){

    loopController1.setReference(666, ControlType.kPosition);

  }  
  
  //Get arm to the 4th stage
  public void lvl4Arm(){
  
    loopController1.setReference(999, ControlType.kPosition);

  }

  }
