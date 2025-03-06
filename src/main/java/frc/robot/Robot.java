
package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.BlinkinLEDController;
import frc.robot.utils.BlinkinLEDController.BlinkinPattern;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Commands.ManualModeCommand;
import frc.robot.Commands.PickupCommand;
import frc.robot.OI.OperatorInterface;
import au.grapplerobotics.CanBridge;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot{
  // private final Joystick driveController;
  // private final Joystick mineController;
  private Command m_autonomousCommand;
  // private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //private ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  //private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  //private processorArmSubsystem processorArmSubsystem = new ProcessorArmSubsystem();
  //private WristSubsystem wristSubsystem = new WristSubsystem();
  private BlinkinLEDController leds = new BlinkinLEDController();
  // private Constants constants = new Constants();
  private SubsystemManager subsystemManager;

  //Commands
  private PickupCommand pickupCommand;
  private ManualModeCommand manualModeCommand;
  

  public Robot(){
    CanBridge.runTCP();
    // instance = this;
    // driveController = new Joystick(0);
    // mineController = new Joystick(1);
  }

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();
    subsystemManager = new SubsystemManager();

    //pickupCommand = new PickupCommand(subsystemManager.getElevatorSubsystem(), subsystemManager.getWristSubsystem());
    // manualModeCommand = new ManualModeCommand();
    OperatorInterface.create(subsystemManager);

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    leds.setPattern(BlinkinPattern.CONFETTI);
  }
  
  @Override
  public void robotPeriodic(){
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // elevatorSubsystem.periodic();
    // //processorArmSubsystem.periodic();
    // climbSubsystem.periodic();
    // wristSubsystem.periodic();
    subsystemManager.getWristSubsystem().periodic();
    subsystemManager.getElevatorSubsystem().periodic();
    subsystemManager.getSwerveSubsystem().periodic();
    // swerveSubsystem.periodic();
    // if(driveController.getRawButtonPressed(7)){
      // swerveSubsystem.zeroGyro();
    // }
    // if(driveController.getRawButtonPressed(8)){
    //   wristSubsystem.reBoot();
    //   // armSubsystem.reBoot();
    //   climbSubsystem.reBoot();
    // }

    //SmartDashboard.putNumber("null", m_robotDrive.imuReadingCache);
    SmartDashboard.putBoolean("manualMode", Universals.manualMode);}
  

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {

  }

  @Override
  public void disabledPeriodic()
  {

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
    
    // double leftTriggerVal2 = mineController.getRawAxis(2);
    // double rightTriggerVal2 = mineController.getRawAxis(3);

    // introduce deadband to keep controller drift from causing issues
    // double driveX = driveController.getRawAxis(1);
    // double driveY = driveController.getRawAxis(0);
    // double driveRot = -driveController.getRawAxis(4);
    // if(Math.abs(driveX) < 0.1){ driveX = 0;}
    // if(Math.abs(driveY) < 0.1){ driveY = 0;}
    // if(Math.abs(driveRot) < 0.1){ driveRot = 0;}
    
    // swerveSubsystem.drive(driveX, driveY, driveRot);

  // climber
  // if(mineController.getRawButton(1)){
  //     climbSubsystem.bringOut();
  // } else if(mineController.getRawButton(4)){
  //   // climbSubsystem.openHand();
  //   // if(climbSubsystem.climbEncoderPos < 589){
  //     if(climbSubsystem.climbEncoderPos > 0){
  //       climbSubsystem.bringIn();
  //     }
  //     else{
        
  //     }
  // } else{
  //   climbSubsystem.stop();
  // }

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
  // if(driveController.getRawButton(1)){
  //   elevatorSubsystem.level = 1.0;
  //   // armSubsystem.armScore();
  //   wristSubsystem.wristScore();
  //   leds.setPattern(BlinkinPattern.RED);
  // } else if(driveController.getRawButton(2)){
  //   elevatorSubsystem.level = 2.0;
  //   // armSubsystem.armAlgae();
  //   wristSubsystem.wristScore();
  //   leds.setPattern(BlinkinPattern.DARK_RED);
  // } else if(driveController.getRawButton(3)){
  //   elevatorSubsystem.level = 3.0;
  //   // armSubsystem.armScore();
  //   wristSubsystem.wristScore();
  //   leds.setPattern(BlinkinPattern.AQUA);
  // } else if(driveController.getRawButton(4)){
  //   elevatorSubsystem.level = 4.0;
  //   // armSubsystem.armScore();
  //   wristSubsystem.lvl4WristScore();
  //   leds.setPattern(BlinkinPattern.HOT_PINK);
  // }


  // if(driveController.getRawButtonPressed(10) && (manualMode == false)){
  //   manualMode = true;
  // } else if(driveController.getRawButtonPressed(10) && (manualMode == true)){
  //   manualMode = false;
  // }
  
  // if(driveController.getRawButtonPressed(10)){
  //   if(manualMode == true){
  //     manualMode = false;
  //   } else {
  //     manualMode = true;
  //   }
  // } 
  // if(manualMode == true){
  //   if(driveController.getRawButton(1)){
  //     elevatorSubsystem.reverseElevate();
  //   } else if(driveController.getRawButton(4)){
  //     elevatorSubsystem.goElevate();
  //   } else{
  //     elevatorSubsystem.stopElevate();
  //   }
  // } else {
  //   elevatorSubsystem.elPIDToLevel();
  // }
  
  //Intake
  // if(driveController.getRawAxis(2) > 0.5){
  //   wristSubsystem.intake(driveController.getRawAxis(2));
  //   wristSubsystem.wristPickup();
  //   elevatorSubsystem.level = 0.0;
  // }
  //   else if(driveController.getRawAxis(3) > 0.5){
  //   wristSubsystem.outake(driveController.getRawAxis(3));
  // } 
  // else if(driveController.getRawButton(5)){
  //   wristSubsystem.intake(0.8);
  // }
  // else {
  //   wristSubsystem.stop();
  // }

  
 //  im programming im haker man i do haks yeahahahahahahahahhh im in the mainframe babyyyyyyyyyyyyyy*/
}

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // m_robotContainer.setDriveMode(); 
  }

  @Override
  public void testPeriodic()
  {
  }

  @Override
  public void simulationInit()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }
}
