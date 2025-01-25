package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem {
  private LaserCan lc;

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
  //lasercan
  public void laserInit() {
    lc = new LaserCan(199);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public void laserPeriodic() {
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
    } else {
      System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
  }

  public void periodic(){
  elencoderPos = leftelEncoder.getPosition();
  SmartDashboard.putNumber("ElEncoder", elencoderPos);
  laserPeriodic();
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
