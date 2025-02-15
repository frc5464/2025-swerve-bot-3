package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ArmSubsystem {
  SparkMax armIntake = new SparkMax(8, MotorType.kBrushless);
  TalonFX armRot = new TalonFX(9);
  SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0;
  double requestPosition = -6;
  double extMaxOutput = 0;
  double extMinOutput = 0;
  PositionVoltage m_request;
  RelativeEncoder armEncoder;
  public double encoderPos;

  public void init(){   
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.24;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    armRot.getConfigurator().apply(slot0Configs);

    // // create a position closed-loop request, voltage output, slot 0 configs
    m_request = new PositionVoltage(0).withSlot(0);
  
  }

  public void periodic(){
    // set position to 10 rotation
    armRot.setControl(m_request.withPosition(requestPosition));

}
//0, 20, 40
  //Drop Coral
  public void dropCoral(){
    armIntake.set(0.3);
  }
  public void retrieveCoral(){

    armIntake.set(-0.3);
  }
  public void stopIntake(){
    armIntake.set(0);
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

  // Get arm to Coral pickup position
  public void armPickup(){
    requestPosition = -4;
  }
  

  //Get arm to Coral scoring position
  public void armScore(){
    requestPosition = -17;
  }
  
  //Get arm to starting position
  public void armStart(){
    requestPosition = -4;
  }

  }
