package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem {
  private LaserCan laserCannon;
  SparkMax leftEl = new SparkMax(5, MotorType.kBrushless);
  SparkMax rightEl = new SparkMax(6, MotorType.kBrushless); //right follows Left
  SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  // SparkClosedLoopController elPID = leftEl.getClosedLoopController();
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0;
  double extMaxOutput = -0.2;
  double extMinOutput = 0.2;
  RelativeEncoder leftelEncoder;
  public double elencoderPos;
  public double targetPosition;
  public double level = 0.0;
  public double lasercanMeasurement;
  boolean laserOk = false;
  PIDController elevatorPid;
  double maxElevatorPower = 0.05;

  public ElevatorSubsystem(){
    elevatorPid = new PIDController(0.04, 10e-2, 0);
    leftelEncoder = leftEl.getEncoder();
    laserInit();
    SparkBaseConfig conf = new SparkMaxConfig();
    conf.openLoopRampRate(0.5);
    leftEl.configure(conf, null, null);

  }

  //lasercan
  public void laserInit() {
    laserCannon = new LaserCan(25);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
    try {
      laserCannon.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCannon.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laserCannon.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      laserOk = true;
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed!" + e);
      laserOk = false;
    }
  }

  public void laserPeriodic() {
    LaserCan.Measurement measurement = laserCannon.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      
      lasercanMeasurement = measurement.distance_mm;
      // System.out.println("The target is" + lasercanMeasurement+ "mm away!");
      laserOk = true;
    } else {
      // System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      laserOk = false;
    }
  }

  public void periodic(){
    elencoderPos = leftelEncoder.getPosition();
    SmartDashboard.putNumber("ElEncoder", elencoderPos);
    SmartDashboard.putNumber("ElLaser", lasercanMeasurement);
    SmartDashboard.putNumber("ElTarget", targetPosition);
    SmartDashboard.putNumber("Offset", targetPosition - lasercanMeasurement);


    laserPeriodic();
  }

  public void goElevate(){
    leftEl.set(0.25);
    // rightEl.set(-0.075);
  }

  public void reverseElevate(){
    leftEl.set(-0.20);
    // rightEl.set(0.05);
  }

  public void stopElevate(){
    leftEl.set(0);
    // rightEl.set(0);
  }
  public void elPIDToLevel(){
    if(level == 0.0){
      targetPosition = 0;
    }
    if(level == 1.0){
      targetPosition = 80;
    }
    
    if(level == 2.0){
      targetPosition = 123;
    }
    if(level == 2.5){
      targetPosition = 478;
    }
    if(level == 3.0){
      targetPosition = 280;
    }

    if(level == 4.0){
      targetPosition  = 697;
    }
    if(laserOk){
      leftEl.set((elevatorPid.calculate(lasercanMeasurement,targetPosition) * maxElevatorPower));
    }
    else{
      leftEl.set(0);
    }
  }
}