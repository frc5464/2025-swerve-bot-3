package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristSubsystem {

  SparkMax armWrist = new SparkMax(7, MotorType.kBrushless);
  SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0;
  double extMaxOutput = -0.2;
  double extMinOutput = 0.2;
  PositionVoltage m_request;
  RelativeEncoder wristEncoder;
  public double wristencoderPos;
  public double counts;
  public int level = 0;

  double requestPosition = 0;

  public void init(){

    sparkMaxConfig.closedLoop
      .p(kP)
      .i(kI)
      .d(kD)
      .outputRange(extMinOutput, extMaxOutput);
    
    armWrist.configure(sparkMaxConfig, null, null);


    m_request = new PositionVoltage(0).withSlot(0);

  }

  public void periodic(){
  wristencoderPos = wristEncoder.getPosition();
  SmartDashboard.putNumber("WristEncoder", wristencoderPos);

  }

  //Flick of da wrist
  public void windUp(){
    armWrist.set(0.2);
  }
  public void windDown(){
    armWrist.set(-0.2);
  }
  public void windStop(){
    armWrist.set(0);
  }

    // Get arm to Coral pickup position
  public void armPickup(){
    requestPosition = 20;
  }
  
  //Get arm to Coral scoring position
  public void armScore(){
    requestPosition = 40;
  }
  
  //Get arm to starting position
  public void armStart(){
    requestPosition = 2;
  }





  


}
