package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem {

  SparkMax leftEl = new SparkMax(30, MotorType.kBrushless);
  SparkMax rightEl = new SparkMax(36, MotorType.kBrushless); //right follows Left
  SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  SparkClosedLoopController elPid = leftEl.getClosedLoopController();
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0;
  double extMaxOutput = -0.2;
  double extMinOutput = 0.2;
  RelativeEncoder leftelEncoder;
  public double elencoderPos;

  public void init(){

    leftelEncoder = leftEl.getEncoder();

    // sparkMaxConfig.closedLoop
    //   .p(kP)
    //   .i(kI)
    //   .d(kD)
    //   .outputRange(extMinOutput, extMaxOutput);
    
    // leftEl.configure(sparkMaxConfig, null, null);

  }

  public void periodic(){
  elencoderPos = leftelEncoder.getPosition();
  SmartDashboard.putNumber("ElEncoder", elencoderPos);
  }

  public void goElevate(){
    leftEl.set(0.15);
    // rightEl.set(-0.075);
  }

  public void reverseElevate(){
    leftEl.set(-0.15);
    // rightEl.set(0.05);
  }

  public void stopElevate(){
    leftEl.set(0);
    // rightEl.set(0);
  }

  //Get arm to the 1st stage
  public void lvl1El(){
    elPid.setReference(0, ControlType.kPosition);
  }
  
  //Get arm to the 2nd stage
  public void lvl2El(){
    elPid.setReference(0, ControlType.kPosition);
  }
  
  //Get arm to the 3rd stage
  public void lvl3El(){
    elPid.setReference(0, ControlType.kPosition);
  }  
  
  //Get arm to the 4th stage
  public void lvl4El(){
    elPid.setReference(0, ControlType.kPosition);
  }
}
