// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
//import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ProcessorArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.io.File;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot{
  private SwerveDrive m_robotDrive;
  private final Joystick driveController;
  private final Joystick mineController;

  private Command m_autonomousCommand;

  // private RobotContainer m_robotContainer;

  private ArmSubsystem armSubsystem;

  private ClimbSubsystem climbSubsystem;
  
  private ElevatorSubsystem elevatorSubsystem;

  private ProcessorArmSubsystem processorArmSubsystem;

  private WristSubsystem wristSubsystem;

  private Constants constants;

  public Robot(){
    // CanBridge.runTCP();
    // instance = this;
    double maxSpeed = Units.feetToMeters(4);
    File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    
    try {
      m_robotDrive = new SwerveParser(directory).createSwerveDrive(
        maxSpeed,
        new Pose2d(
          new Translation2d(1, 4),
          Rotation2d.fromDegrees(0)
        )
      );
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    driveController = new Joystick(0);
    mineController = new Joystick(1);
  }
  


  
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
    processorArmSubsystem = new ProcessorArmSubsystem();
    constants = new Constants();
    armSubsystem = new ArmSubsystem();
    wristSubsystem = new WristSubsystem();
    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    //disabledTimer = new Timer();
    
    armSubsystem.init();
    elevatorSubsystem.init();
    processorArmSubsystem.init();
    climbSubsystem.init();
    wristSubsystem.init();

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
    processorArmSubsystem.periodic();
    armSubsystem.periodic();
    climbSubsystem.periodic();
    wristSubsystem.periodic();
    if(driveController.getRawButtonPressed(8)){
      wristSubsystem.reBoot();
      armSubsystem.reBoot();

    }
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

    double leftstickval = driveController.getRawAxis(0);
    double rightstickval = driveController.getRawAxis(0);
    double leftTriggerVal = driveController.getRawAxis(2);
    double rightTriggerVal = driveController.getRawAxis(3);
    double leftTriggerVal2 = mineController.getRawAxis(2);
    double rightTriggerVal2 = mineController.getRawAxis(3);

    // introduce deadband to keep controller drift from causing issues
    double driveX = driveController.getRawAxis(1);
    double driveY = driveController.getRawAxis(0);
    double driveRot =  driveController.getRawAxis(4);
    if(Math.abs(driveX) < 0.1){ driveX = 0;}
    if(Math.abs(driveY) < 0.1){ driveY = 0;}
    if(Math.abs(driveRot) < 0.1){ driveRot = 0;}
    
    m_robotDrive.drive(new Translation2d(driveX,driveY),driveRot, false, false);

 
  

  // climber
  if(mineController.getRawAxis(5) < -0.5){
    climbSubsystem.closeHand();
  } else if(mineController.getRawAxis(5) > 0.5){
    climbSubsystem.openHand();
  } else{
    climbSubsystem.stopHand();
  }

  // Processor Rotation
  if(mineController.getRawButton(5)){
    if(processorArmSubsystem.procrotEncoderPos > 5){
    processorArmSubsystem.downrot_procarm();}
  } else if(mineController.getRawButton(6)){
    if(processorArmSubsystem.procrotEncoderPos < 559){
      processorArmSubsystem.rot_procarm();
    }
  } else {
    
  }
  
  // Processor Roll (Int_Out)
  if(mineController.getRawAxis(2) > 0.05){
    processorArmSubsystem.intake(leftTriggerVal2);
  } else if(mineController.getRawAxis(3) > 0.05){
    processorArmSubsystem.outake(rightTriggerVal2);
  } else {
    processorArmSubsystem.stoprot();
  }

  // Elevator level selector
  if(driveController.getRawButton(1)){
    elevatorSubsystem.level = 1.0;
    armSubsystem.armScore();
    wristSubsystem.wristScore();
  } else if(driveController.getRawButton(2)){
    elevatorSubsystem.level = 2.0;
    armSubsystem.armScore();
    wristSubsystem.wristScore();
  } else if(driveController.getRawButton(3)){
    elevatorSubsystem.level = 3.0;
    armSubsystem.armScore();
    wristSubsystem.wristScore();
  } else if(driveController.getRawButton(4)){
    elevatorSubsystem.level = 4.0;
    armSubsystem.armScore();
    wristSubsystem.lvl4WristScore();
  }
  
  //Intake
  if(driveController.getRawAxis(2) > 0.5){
    elevatorSubsystem.level = 2.5;
    armSubsystem.armPickup();
    wristSubsystem.wristPickup();
    armSubsystem.retrieveCoral(0.5);
  } else {
    armSubsystem.stopIntake();
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
