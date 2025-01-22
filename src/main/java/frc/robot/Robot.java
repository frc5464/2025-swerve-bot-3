// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ProcessorArmSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot{

  private static Robot instance;

  private Command m_autonomousCommand;

  // private RobotContainer m_robotContainer;

  private ArmSubsystem armSubsystem;

  // private ClimbSubsystem climbSubsystem;
  
  private ElevatorSubsystem elevatorSubsystem;

  private ProcessorArmSubsystem processorArmSubsystem;

  private Constants constants;

  private Timer disabledTimer;

  // public SparkMax frontRightDrive = new SparkMax(35, MotorType.kBrushless);
  // public SparkMax frontRightTurn = new SparkMax(34, MotorType.kBrushless);
  // public SparkMax frontLeftDrive = new SparkMax(32, MotorType.kBrushless);
  // public SparkMax frontLeftTurn = new SparkMax(33, MotorType.kBrushless);
  // public SparkMax backRightDrive = new SparkMax(37, MotorType.kBrushless);
  // public SparkMax backRightTurn = new SparkMax(36, MotorType.kBrushless);
  // public SparkMax backLeftDrive = new SparkMax(31, MotorType.kBrushless);
  // public SparkMax backLeftTurn = new SparkMax(30, MotorType.kBrushless);

  // public RelativeEncoder frontRightDriveRelativeEncoder;
  // public RelativeEncoder frontRightTurnRelativeEncoder;
  // public RelativeEncoder frontLeftDriveRelativeEncoder;
  // public RelativeEncoder frontLeftTurnRelativeEncoder;
  // public RelativeEncoder backRightDriveRelativeEncoder;
  // public RelativeEncoder backRightTurnRelativeEncoder;
  // public RelativeEncoder backleftDriveRelativeEncoder;
  // public RelativeEncoder backleftTurnRelativeEncoder;

  
  // public LaserCan laserCan;
  
  public Robot(){
    // CanBridge.runTCP();
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }


  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();
    //armSubsystem = new ArmSubsystem();
    // climbSubsystem = new ClimbSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    processorArmSubsystem = new ProcessorArmSubsystem();
    constants = new Constants();
    armSubsystem = new ArmSubsystem();
    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    
    armSubsystem.init();
    elevatorSubsystem.init();
    processorArmSubsystem.init();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // frontRightDriveRelativeEncoder = frontRightDrive.getEncoder();
    // frontRightTurnRelativeEncoder = frontRightTurn.getEncoder();
    // frontLeftDriveRelativeEncoder = frontLeftDrive.getEncoder();
    // frontLeftTurnRelativeEncoder = frontLeftTurn.getEncoder();
    // backRightDriveRelativeEncoder = backRightDrive.getEncoder();
    // backRightTurnRelativeEncoder = backRightTurn.getEncoder();
    // backleftDriveRelativeEncoder = backLeftDrive.getEncoder();
    // backleftTurnRelativeEncoder = backLeftTurn.getEncoder();

    //For Debugging
    // while (true);
      
  

    // laserCan = new LaserCan(0);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
    // try {
    //   laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
    //   laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
    //   laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    // } catch (ConfigurationFailedException e) {
    //   System.out.println("Configuration failed! " + e);
    // }

  }

  Joystick driverController = new Joystick(0);
  Joystick mineController = new Joystick(1);
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic(){
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    elevatorSubsystem.periodic();
    processorArmSubsystem.periodic();
    armSubsystem.periodic();
    
    // SmartDashboard.putNumber("FRdEncoder", frontRightDriveRelativeEncoder.getPosition());
    // SmartDashboard.putNumber("FRtEncoder", frontRightTurnRelativeEncoder.getPosition());
    // SmartDashboard.putNumber("FLdEncoder", frontLeftDriveRelativeEncoder.getPosition());
    // SmartDashboard.putNumber("FLtEncoder", frontLeftTurnRelativeEncoder.getPosition());
    // SmartDashboard.putNumber("BRdEncoder", backRightDriveRelativeEncoder.getPosition());
    // SmartDashboard.putNumber("BRtEncoder", backRightTurnRelativeEncoder.getPosition());
    // SmartDashboard.putNumber("BLdEncoder", backleftDriveRelativeEncoder.getPosition());
    // SmartDashboard.putNumber("BLtEncoder", backleftTurnRelativeEncoder.getPosition());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    // m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      // m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    // m_robotContainer.setMotorBrake(true);
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
    // m_robotContainer.setDriveMode();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic(){

    double leftstickval = driverController.getRawAxis(0);
    double rightstickval = driverController.getRawAxis(0);

    double leftTriggerVal = mineController.getRawAxis(2);
    double rightTriggerVal = mineController.getRawAxis(3);

      if(mineController.getRawButton(6)){
        armSubsystem.rotArm();
      } else if(mineController.getRawButton(5)){
        armSubsystem.revrotArm();
      } else {
        armSubsystem.stopArm();
      } 
      
  
    // if(driverController.getRawButton(1)){
    //   // armSubsystem.lvl1Arm();
    //   elevatorSubsystem.lvl1El();
    // } else if(driverController.getRawButton(2)){
    //   // armSubsystem.lvl3Arm();
    //   elevatorSubsystem.lvl3El();
    // } else if(driverController.getRawButton(3)){
    //   // armSubsystem.lvl2Arm();
    //   elevatorSubsystem.lvl2El();
    // } else if(driverController.getRawButton(4)){
    //   // armSubsystem.lvl4Arm();
    //   elevatorSubsystem.lvl4El();
    // } else{
    //   // armSubsystem.stopArm();
    //   elevatorSubsystem.stopElevate();
    // }

  // Processor Rotation
  if(mineController.getRawButton(1)){
    processorArmSubsystem.downrot_procarm();
  } else if(mineController.getRawButton(4)){
    processorArmSubsystem.rot_procarm();
  } else{
    processorArmSubsystem.stoprot_procarm();
  }
  // Processor Roll (Int_Out)
  if(mineController.getRawAxis(2) > 0.1){
    processorArmSubsystem.roll_procarm();
  } else if(mineController.getRawAxis(3) < 0.1){
    processorArmSubsystem.revroll_procarm();
  } else{
    processorArmSubsystem.stoproll_procarm();
  }
  
  // Elevator
  if(driverController.getPOV() == 0){
    if(elevatorSubsystem.elencoderPos < 59){
      elevatorSubsystem.goElevate();}
  } else if(driverController.getPOV() == 180){
    if(elevatorSubsystem.elencoderPos > 1){
      elevatorSubsystem.reverseElevate();}
    } 
  else{
    elevatorSubsystem.stopElevate();
  }

  
}

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // m_robotContainer.setDriveMode();
    
  }

  /**
  //  * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
