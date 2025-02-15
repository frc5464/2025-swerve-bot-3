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
  //   // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
  //   // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  //   // slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
  //   // slot0Configs.kI = 0; // no output for integrated error
  //   // slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    armRot.getConfigurator().apply(slot0Configs);

    // // create a position closed-loop request, voltage output, slot 0 configs
    m_request = new PositionVoltage(0).withSlot(0);


  //   // Trapezoid profile with max velocity 80 rps, max accel 160 rps/s
  // final TrapezoidProfile m_profile = new TrapezoidProfile(
  //  new TrapezoidProfile.Constraints(80, 160)
  // );
  // // Final target of 200 rot, 0 rps
  // TrapezoidProfile.State m_goal = new TrapezoidProfile.State(200, 0);
  // TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();



  // // calculate the next profile setpoint
  // m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

  // // send the request to the device
  // m_request.Position = m_setpoint.position;
  // m_request.Velocity = m_setpoint.velocity;
  // armRot.setControl(m_request);

  //Encoderstuff
  // sparkMaxConfig.closedLoop
  //   .p(kP)
  //   .i(kI)
  //   .d(kD)
  //   .outputRange(extMinOutput, extMaxOutput);
  
  // armRot.configure(sparkMaxConfig, null, null);
  
  }

  public void periodic(){

    // set position to 10 rotation
    armRot.setControl(m_request.withPosition(requestPosition));


  // armEncoderPos = armEncoder.getPosition();
  // SmartDashboard.putNumber("Encoder", armEncoderPos);

}
//0, 20, 40
  //Drop Coral
  public void dropCoral(double axi2){
    armCoral.set(axi2);
  }
  public void retrieveCoral(double axi3){

    armIntake.set(-axi3);
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
