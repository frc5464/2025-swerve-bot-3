package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Universals;

public class ElevatorSubsystem {
  private LaserCan laserCannon;
  SparkMax leftEl = new SparkMax(5, MotorType.kBrushless);
  SparkMax rightEl = new SparkMax(6, MotorType.kBrushless); //right follows Left
  SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  // SparkClosedLoopController elPID = leftEl.getClosedLoopController();
  double kP = 0.04;
  double kI = 10e-3;
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
  double laser;
  double encoder;
  public ElevatorSubsystem(){
    elevatorPid = new PIDController(kP, kI, kD);
    
    leftelEncoder = leftEl.getEncoder();
    laserInit();

    // Make things buttery smooth
    SparkBaseConfig conf = new SparkMaxConfig();
    conf.idleMode(IdleMode.kCoast);
    conf.openLoopRampRate(0.5);

    leftEl.configure(conf, null, null);

    // Follow the yellow brick road (left elevator)
    SparkBaseConfig conf2 = new SparkMaxConfig();
    conf2.follow(5, true);
    conf2.idleMode(IdleMode.kCoast);
    rightEl.configure(conf2, null, null);

    // CanBridge.runTCP();
    SmartDashboard.putNumber("elP", kP);
    SmartDashboard.putNumber("elI", kI);
    SmartDashboard.putNumber("elD", kD);
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

  private void checkForPidChanges(){
    double newP = SmartDashboard.getNumber("elP", -1);
    double newI = SmartDashboard.getNumber("elI", -1);
    double newD = SmartDashboard.getNumber("elD", -1);
    // System.out.println(newP);
    if(newP != kP){
      kP = newP;
      elevatorPid.setP(newP);
      System.out.println("New P parameter!");
    }
    if(newI != kI){
      kI = newI;
      elevatorPid.setI(newI);
      System.out.println("New I parameter!");
    }
    if(newD != kD){
      kD = newD;
      elevatorPid.setD(newD);
      System.out.println("New D parameter!");
    }
  }

  public void periodic(){
    elencoderPos = leftelEncoder.getPosition();
    SmartDashboard.putNumber("ElEncoder", elencoderPos);
    SmartDashboard.putNumber("ElLaser", lasercanMeasurement);
    SmartDashboard.putNumber("ElTarget", targetPosition);
    SmartDashboard.putNumber("ElError", targetPosition - lasercanMeasurement);

    checkForPidChanges();
    
    if(lasercanMeasurement < 15){
      Universals.zoom = true;
    } else {
      Universals.zoom = false;
    }
    if(Universals.manualMode == false){
      elPIDToLevel();
    }

    laserPeriodic();
  }

  public void goElevate(){
    leftEl.set(0.25);
    // rightEl.set(-0.075);
  }

  public void reverseElevate(){
    leftEl.set(-0.20);
    // rightEl.set(0.05);hello
  }

  public void stopElevate(){
    leftEl.set(0);
    // rightEl.set(0);
  }
    public boolean emergencyStop(double laser, double encoder){
      if(laser < 80 && encoder > 50){
        return true;
      } else {
        return false;
      }
    }
  public void elPIDToLevel(){
    if(level == 0.0){
      targetPosition = 0;
    }
    if(level == 1.0){
      targetPosition = 8;
    }
    
    if(level == 2.0){
      targetPosition = 147;
    }
    if(level == 2.5){
      targetPosition = 478;
    }
    if(level == 3.0){
      targetPosition = 342;
    }

    if(level == 4.0){
      targetPosition  = 720;
    }
    if(laserOk){
      if(emergencyStop(lasercanMeasurement, elencoderPos)){
        stopElevate();
      } else {
        leftEl.set((elevatorPid.calculate(elencoderPos * 11.7,targetPosition) * maxElevatorPower));
      }
      //leftEl.set((elevatorPid.calculate(lasercanMeasurement,targetPosition) * maxElevatorPower));
      
    }
    else{
      leftEl.set(0);
    }
  }

  public void reBoot(){
    leftelEncoder.setPosition(0);
  }
}