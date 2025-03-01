// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.subsystems.ArmSubsystem;
//import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ProcessorArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

// import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.io.File;

import au.grapplerobotics.CanBridge;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot{
  
  private final Joystick driveController;
  private final Joystick mineController;

  private Command m_autonomousCommand;

  // private RobotContainer m_robotContainer;

  private SwerveSubsystem swerveSubsystem;

  //private ArmSubsystem armSubsystem;

  private ClimbSubsystem climbSubsystem;
  
  private ElevatorSubsystem elevatorSubsystem;

  //private ProcessorArmSubsystem processorArmSubsystem;

  private WristSubsystem wristSubsystem;

  private Constants constants;

  public Robot(){
    CanBridge.runTCP();
    // instance = this;

    driveController = new Joystick(0);
    mineController = new Joystick(1);
    
  }
  
  boolean manualMode = false;

  
  // public static Robot getInstance()
  // {
  //   return instance;
  // }


  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();
    climbSubsystem = new ClimbSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    //processorArmSubsystem = new ProcessorArmSubsystem();
    constants = new Constants();
    // armSubsystem = new ArmSubsystem();
    wristSubsystem = new WristSubsystem();
    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    //disabledTimer = new Timer();
    swerveSubsystem = new SwerveSubsystem();
  

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

  }
  
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
    //processorArmSubsystem.periodic();
    //armSubsystem.periodic();
    climbSubsystem.periodic();
    wristSubsystem.periodic();

    // if(driveController.getRawButton(6)){
      
    // }

    if(driveController.getRawButtonPressed(7)){
      swerveSubsystem.zeroGyro();
    }
    if(driveController.getRawButtonPressed(8)){
      wristSubsystem.reBoot();
      // armSubsystem.reBoot();
      climbSubsystem.reBoot();
    }

    //SmartDashboard.putNumber("null", m_robotDrive.imuReadingCache);
    SmartDashboard.putBoolean("manualMode", manualMode);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    // m_robotContainer.setMotorBrake(false);
    // disabledTimer.reset();
    // disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    // if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    // {
    //   m_robotContainer.setMotorBrake(false);
    //   disabledTimer.stop();
    // }
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
    
    double leftTriggerVal2 = mineController.getRawAxis(2);
    double rightTriggerVal2 = mineController.getRawAxis(3);

    // introduce deadband to keep controller drift from causing issues
    double driveX = driveController.getRawAxis(1);
    double driveY = driveController.getRawAxis(0);
    double driveRot = -driveController.getRawAxis(4);
    if(Math.abs(driveX) < 0.1){ driveX = 0;}
    if(Math.abs(driveY) < 0.1){ driveY = 0;}
    if(Math.abs(driveRot) < 0.1){ driveRot = 0;}
    
    swerveSubsystem.drive(driveX, driveY, driveRot);

  // climber
  if(mineController.getRawButton(1)){
      climbSubsystem.bringOut();
  } else if(mineController.getRawButton(4)){
    // climbSubsystem.openHand();
    // if(climbSubsystem.climbEncoderPos < 589){
      if(climbSubsystem.climbEncoderPos > 0){
        climbSubsystem.bringIn();
      }
      else{
        
      }
  } else{
    climbSubsystem.stop();
  }

  // Processor Rotation
  // if(mineController.getRawButton(5)){
  //   if(processorArmSubsystem.procrotEncoderPos > 5){
  //   processorArmSubsystem.downrot_procarm();}
  // } else if(mineController.getRawButton(6)){
  //   if(processorArmSubsystem.procrotEncoderPos < 559){
  //     processorArmSubsystem.rot_procarm();
  //   }
  // } else {
    
  // }
  
  // // Processor Roll (Int_Out)
  // if(mineController.getRawAxis(2) > 0.05){
  //   processorArmSubsystem.intake(leftTriggerVal2);
  // } else if(mineController.getRawAxis(3) > 0.05){
  //   processorArmSubsystem.outake(rightTriggerVal2);
  // } else {
  //   processorArmSubsystem.stoprot();
  // }

  
  // if(driveController.getPOV() == 0){
  //   armSubsystem.armScore();
  //   wristSubsystem.wristScore();
  // }

  // if(driveController.getRawButton(6)){
  //   armSubsystem.armAlgae();
  //   wristSubsystem.wristAlgae();
  // }

  //Elevator level selector
  if(driveController.getRawButton(1)){
    elevatorSubsystem.level = 1.0;
    // armSubsystem.armScore();
    wristSubsystem.wristScore();
  } else if(driveController.getRawButton(2)){
    elevatorSubsystem.level = 2.0;
    // armSubsystem.armAlgae();
    wristSubsystem.wristScore();
  } else if(driveController.getRawButton(3)){
    elevatorSubsystem.level = 3.0;
    // armSubsystem.armScore();
    wristSubsystem.wristScore();
  } else if(driveController.getRawButton(4)){
    elevatorSubsystem.level = 4.0;
    // armSubsystem.armScore();
    wristSubsystem.lvl4WristScore();
  }

  // if(driveController.getRawButtonPressed(10) && (manualMode == false)){
  //   manualMode = true;
  // } else if(driveController.getRawButtonPressed(10) && (manualMode == true)){
  //   manualMode = false;
  // }
  
  if(driveController.getRawButtonPressed(10)){
    if(manualMode == true){
      manualMode = false;
    } else {
      manualMode = true;
    }
  } 
  if(manualMode == true){
    if(driveController.getRawButton(1)){
      elevatorSubsystem.reverseElevate();
    } else if(driveController.getRawButton(4)){
      elevatorSubsystem.goElevate();
    } else{
      elevatorSubsystem.stopElevate();
    }
  } else {
    elevatorSubsystem.elPIDToLevel();
  }

  

  // if(driveController.getRawButton(5)){
  //   elevatorSubsystem.level = 2.5;
  //   armSubsystem.armPickup();
  //   wristSubsystem.wristPickup();
  // }
  
  //Intake
  if(driveController.getRawAxis(2) > 0.5){
    wristSubsystem.intake(driveController.getRawAxis(2));
    wristSubsystem.wristPickup();
    elevatorSubsystem.level = 0.0;
  }
    else if(driveController.getRawAxis(3) > 0.5){
    wristSubsystem.outake(driveController.getRawAxis(3));
  } else {
    wristSubsystem.stop();
  }

  
 //  im programming im haker man i do haks yeahahahahahahahahhh im in the mainframe babyyyyyyyyyyyyyy*/
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
