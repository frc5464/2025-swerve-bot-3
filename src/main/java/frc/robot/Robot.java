
package frc.robot;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utils.BlinkinLEDController;
import frc.robot.utils.BlinkinLEDController.BlinkinPattern;
import frc.robot.Commands.GyroReset;
import frc.robot.Commands.IntakeOutakeCommand;
import frc.robot.Commands.PickupCommand;
import frc.robot.Commands.ToLevelCommand;
import frc.robot.Commands.WaitCommand;
import frc.robot.Commands.ZeroCommand;
import frc.robot.OI.OperatorInterface;
// import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAGHHHHHH
import frc.robot.subsystems.WristSubsystem;

public class Robot extends TimedRobot{

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private BlinkinLEDController leds = new BlinkinLEDController();
  private SubsystemManager subsystemManager;

  private IntakeOutakeCommand intakeCommand;
  private IntakeOutakeCommand outakeCommand;
  private ToLevelCommand toLevelCommand;
  private ToLevelCommand toLevel1;
  private ToLevelCommand toLevel2;
  private ToLevelCommand toLevel3;
  private ToLevelCommand toLevel4;
  private PickupCommand pickupCommand;
  private GyroReset gyroReset;
  private ZeroCommand zeroCommand;

  //Building the Autonomous Chooser (May be unnecessary with AutoBuilder)
  // private String auto_selected;
  // private final SendableChooser<String> auto_chooser = new SendableChooser<>();
  // public static String kB2_Left;
  // public static String kB2_Right;

  private String wait_selected;
  private final SendableChooser<String> wait_chooser = new SendableChooser<>();
  private static final String k0Seconds = "0Seconds";
  private static final String k1Seconds = "1Seconds";
  private static final String k2Seconds = "2Seconds";
  private static final String k3Seconds = "3Seconds";
  private static final String k4Seconds = "4Seconds";
  private static final String k5Seconds = "5Seconds";
  private static final String k8Seconds = "8Seconds";

  @Override
  public void robotInit() {
    subsystemManager = new SubsystemManager();
    
    ElevatorSubsystem elevator = subsystemManager.getElevatorSubsystem();
    WristSubsystem wrist = subsystemManager.getWristSubsystem();
    //ClimbSubsystem climb = subsystemManager.getClimbSubsystem();

    pickupCommand = new PickupCommand(elevator, wrist);
    toLevel1 = new ToLevelCommand(elevator,1.0, wrist, 16);
    toLevel2 = new ToLevelCommand(elevator, 2.0, wrist, 16);
    toLevel3 = new ToLevelCommand(elevator, 3.0, wrist, 16);
    toLevel4 = new ToLevelCommand(elevator, 4.0, wrist, 19);
    intakeCommand = new IntakeOutakeCommand(wrist, true);
    outakeCommand = new IntakeOutakeCommand(wrist, false);
    
    NamedCommands.registerCommand("IntakeCommand", intakeCommand);
    NamedCommands.registerCommand("OutakeCommand", outakeCommand);
    NamedCommands.registerCommand("ToLevelCommand", toLevelCommand);
    NamedCommands.registerCommand("PickupCommand", pickupCommand);
    NamedCommands.registerCommand("GyroReset", gyroReset);
    NamedCommands.registerCommand("ZeroCommand", zeroCommand);
    NamedCommands.registerCommand("toLevel1", toLevel1);
    NamedCommands.registerCommand("toLevel2", toLevel2);
    NamedCommands.registerCommand("toLevel3", toLevel3);
    NamedCommands.registerCommand("toLevel4", toLevel4);



    wait_chooser.addOption("0Seconds", k0Seconds);
    wait_chooser.setDefaultOption("0Seconds", k0Seconds);
    wait_chooser.addOption("1Seconds", k1Seconds);
    wait_chooser.addOption("2Seconds", k2Seconds);
    wait_chooser.addOption("3Seconds", k3Seconds);
    wait_chooser.addOption("4Seconds", k4Seconds);
    wait_chooser.addOption("5Seconds", k5Seconds);
    wait_chooser.addOption("k8Seconds", k8Seconds);
    SmartDashboard.putData("Wait choices", wait_chooser);

    // auto_chooser.addOption("B2_Left", kB2_Left);
    // auto_chooser.addOption("B2_Right", kB2_Right);
    // SmartDashboard.putData("Auto choices", auto_chooser);
    

    OperatorInterface.create(subsystemManager);
    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    leds.setPattern(BlinkinPattern.CONFETTI);

    m_robotContainer = new RobotContainer();
  }
  
  @Override
  public void robotPeriodic(){
    CommandScheduler.getInstance().run();
    // elevatorSubsystem.periodic();
    // //processorArmSubsystem.periodic();
    // climbSubsystem.periodic();
    // wristSubsystem.periodic();
    // subsystemManager.getVisionSubsystem().periodic();
    subsystemManager.getWristSubsystem().periodic();
    subsystemManager.getElevatorSubsystem().periodic();
    subsystemManager.getSwerveSubsystem().periodic();
    // subsystemManager.getProcessorArmSubsystem().periodic();
    // swerveSubsystem.periodic();
    SmartDashboard.putBoolean("manualMode", Universals.manualMode);

  }
  
  @Override
  public void disabledInit()
  {

  }

  @Override
  public void disabledPeriodic(){}

  @Override
  public void autonomousInit(){

    wait_selected = wait_chooser.getSelected();

    switch(wait_selected){  
        case k0Seconds:
        Universals.wait = 0;
        break;

        case k1Seconds:
        Universals.wait = 1;
        break;

        case k2Seconds:
        Universals.wait = 2;
        break;

        case k3Seconds:
        Universals.wait = 3;
        break;

        default:
        Universals.wait = 0;
        break;
    }

    // Sequential grouping which crashes the code on the 2nd go through auto
    // SequentialCommandGroup fullAuto = new SequentialCommandGroup();
    // fullAuto.addCommands(new WaitCommand(Universals.wait));
    // fullAuto.addCommands(m_robotContainer.getAutonomousCommand());
    // // schedule the autonomous command (example)
    // if (fullAuto != null) {
    //   fullAuto.schedule();
    // }    

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }   

  }

  @Override
  public void autonomousPeriodic(){}

  @Override
  public void teleopInit(){
    if (m_autonomousCommand != null){
      m_autonomousCommand.cancel();
    } else{
      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Override
  public void teleopPeriodic(){
 //  im programming im haker man i do haks yeahahahahahahahahhh im in the mainframe babyyyyyyyyyyyyyy*/
}

  @Override
  public void testInit()
  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic(){}

  @Override
  public void simulationInit(){}

  @Override
  public void simulationPeriodic(){}
}
