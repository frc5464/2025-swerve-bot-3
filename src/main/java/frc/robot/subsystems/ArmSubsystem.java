package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem {
  
  SparkMax armWrist = new SparkMax(7, MotorType.kBrushless);
<<<<<<< Updated upstream
<<<<<<< Updated upstream
  SparkMax armCoral = new SparkMax(8, MotorType.kBrushless);
=======
  SparkMax armIntake = new SparkMax(8, MotorType.kBrushless);
>>>>>>> Stashed changes
=======
  SparkMax armIntake = new SparkMax(8, MotorType.kBrushless);
>>>>>>> Stashed changes
  //SparkMax armRot = new SparkMax(34, MotorType.kBrushless);
  TalonFX armRot = new TalonFX(9);
  SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  
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
    
    var slot0Configs = new Slot0Configs();
  //   // slot0Configs.kP = 2.4;
  //   // slot0Configs.kI = 0;
  //   // slot0Configs.kD = 0.1;

  //   // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
  //   // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  //   // slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
  //   // slot0Configs.kI = 0; // no output for integrated error
  //   // slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

  //   // armRot.getConfigurator().apply(slot0Configs);

  //   // Trapezoid profile with max velocity 80 rps, max accel 160 rps/s
  // final TrapezoidProfile m_profile = new TrapezoidProfile(
  //  new TrapezoidProfile.Constraints(80, 160)
  // );
  // // Final target of 200 rot, 0 rps
  // TrapezoidProfile.State m_goal = new TrapezoidProfile.State(200, 0);
  // TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  // // create a position closed-loop request, voltage output, slot 0 configs
  // final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  // // calculate the next profile setpoint
  // m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

  // // send the request to the device
  // m_request.Position = m_setpoint.position;
  // m_request.Velocity = m_setpoint.velocity;
  // armRot.setControl(m_request);

  //armEncoder = armRot.getEncoder();
  //Encoderstuff
  // sparkMaxConfig.closedLoop
  //   .p(kP)
  //   .i(kI)
  //   .d(kD)
  //   .outputRange(extMinOutput, extMaxOutput);
  
  // armRot.configure(sparkMaxConfig, null, null);
  
  }

  public void periodic(){
  // encoderPos = armEncoder.getPosition();
  SmartDashboard.putNumber("Encoder", encoderPos);
  }

  

  //Drop Coral
  public void dropCoral(double axi2){
    armIntake.set(axi2);
  }
  public void retrieveCoral(double axi3){
    armIntake.set(-axi3);
  }
<<<<<<< Updated upstream
<<<<<<< Updated upstream
  
=======
=======
>>>>>>> Stashed changes
  public void stopIntake(){
    armIntake.set(0);
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

<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
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
  // public void lvl1Arm(){

  //   loopController1.setReference(333, ControlType.kPosition);

  // }
  
  // //Get arm to the 2nd stage
  // public void lvl2Arm(){

  //   loopController1.setReference(444, ControlType.kPosition);

  // }
  
  // //Get arm to the 3rd stage
  // public void lvl3Arm(){

  //   loopController1.setReference(666, ControlType.kPosition);

  // }  
  
  // //Get arm to the 4th stage
  // public void lvl4Arm(){
  
  //   loopController1.setReference(999, ControlType.kPosition);

  // }

  }
