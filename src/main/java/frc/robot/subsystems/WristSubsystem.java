package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.PositionVoltage;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristSubsystem {

  // SparkMax armWrist = new SparkMax(7, MotorType.kBrushless);
  // SparkMaxConfig sparkMaxConfig;
  // double kP = 0;
  // double kI = 0;
  // double kD = 0;
  // double kIz = 0;
  // double kFF = 0;
  // double extMaxOutput = -0.2;
  // double extMinOutput = 0.2;
  // RelativeEncoder wristEncoder;
  // public double wristencoderPos;
  // public double counts;
  public int level = 0;

  double targetPosition = 0.5;

  // private SparkMaxConfig motorConfig;
  // private SparkClosedLoopController closedLoopController;

  private SparkMax motor;
  private SparkMax motor2;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  public WristSubsystem(){
    motor = new SparkMax(7, MotorType.kBrushless);
    motor2 = new SparkMax(8, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.3)
        .i(0)
        .d(0)
        .outputRange(-0.3, 0.3);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

// used https://github.com/REVrobotics/REVLib-Examples/blob/main/Java/SPARK/Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java as example


  public void periodic(){
  SmartDashboard.putNumber("WristEncoder", encoder.getPosition());
  closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  //Flick of da wrist
  public void windUp(){
    motor.set(0.2);
  }
  public void windDown(){
    motor.set(-0.2);
  }
  public void windStop(){
    motor.set(0);
  }

    // Get arm to Coral pickup position
  public void wristPickup(){
    targetPosition = 0.5    ;
  }
  
  //Get arm to Coral scoring position
  public void wristScore(){
    targetPosition = 16;
  }

  public void lvl4WristScore(){
    targetPosition = 19;
  }

  public void wristAlgae(){
    targetPosition = 25;
  }
  
  public void intake(double something){
    motor2.set(something * 0.5);
  }
  public void outake(double something){
    motor2.set(-something * 0.5);
  }
  public void stop(){
    motor2.set(0);
  }
  //Get arm to starting position
  public void wristStart(){
    targetPosition = 2;
  }

  

  public void reBoot(){
    encoder.setPosition(0);
  }




  


}
