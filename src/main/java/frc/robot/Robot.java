
package frc.robot;
import java.io.WriteAbortedException;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
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
import frc.robot.Commands.ZeroCommand;
import frc.robot.OI.OperatorInterface;
// import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class Robot extends TimedRobot{
  private Command m_autonomousCommand;
  private BlinkinLEDController leds = new BlinkinLEDController();
  private SubsystemManager subsystemManager;
  private Command autonomousCommand;

  //Commands to register in Path Planner
  private IntakeOutakeCommand intakeOutakeCommand;
  private ToLevelCommand toLevelCommand;
  private ToLevelCommand toLevel1;
  private ToLevelCommand toLevel2;
  private ToLevelCommand toLevel3;
  private ToLevelCommand toLevel4;
  private PickupCommand pickupCommand;
  private GyroReset gyroReset;
  private ZeroCommand zeroCommand;


  @Override
  public void robotInit() {
    // m_robotContainer = new RobotContainer();
    subsystemManager = new SubsystemManager();
    ElevatorSubsystem elevator = subsystemManager.getElevatorSubsystem();
    WristSubsystem wrist = subsystemManager.getWristSubsystem();
    //ClimbSubsystem climb = subsystemManager.getClimbSubsystem();

    pickupCommand = new PickupCommand(elevator, wrist);
    toLevel1 = new ToLevelCommand(elevator,1.0, wrist, 16);
    toLevel2 = new ToLevelCommand(elevator, 2.0, wrist, 16);
    toLevel3 = new ToLevelCommand(elevator, 3.0, wrist, 16);
    toLevel4 = new ToLevelCommand(elevator, 4.0, wrist, 19);

    NamedCommands.registerCommand("IntakeOutake", intakeOutakeCommand);
    NamedCommands.registerCommand("ToLevelCommand", toLevelCommand);
    NamedCommands.registerCommand("PickupCommand", pickupCommand);
    NamedCommands.registerCommand("GyroReset", gyroReset);
    NamedCommands.registerCommand("ZeroCommamnd", zeroCommand);
    NamedCommands.registerCommand("toLevel1", toLevel1);
    NamedCommands.registerCommand("toLevel2", toLevel2);
    NamedCommands.registerCommand("toLevel3", toLevel3);
    NamedCommands.registerCommand("toLevel4", toLevel4);


    OperatorInterface.create(subsystemManager);
    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    leds.setPattern(BlinkinPattern.CONFETTI);

    
  }
  
  @Override
  public void robotPeriodic(){
    CommandScheduler.getInstance().run();
    // elevatorSubsystem.periodic();
    // //processorArmSubsystem.periodic();
    // climbSubsystem.periodic();
    // wristSubsystem.periodic();
    subsystemManager.getVisionSubsystem().periodic();
    subsystemManager.getWristSubsystem().periodic();
    subsystemManager.getElevatorSubsystem().periodic();
    subsystemManager.getSwerveSubsystem().periodic();
    // subsystemManager.getProcessorArmSubsystem().periodic();
    // swerveSubsystem.periodic();
    SmartDashboard.putBoolean("manualMode", Universals.manualMode);}
  
  @Override
  public void disabledInit()
  {

  }

  @Override
  public void disabledPeriodic(){}

  @Override
  public void autonomousInit()
  {
    SequentialCommandGroup auto = new SequentialCommandGroup();
    auto.addCommands(new GyroReset(subsystemManager.getSwerveSubsystem()));
    auto.addCommands(new PathPlannerAuto("B2_Left"));

    m_autonomousCommand = auto;

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic(){}

  @Override
  public void teleopInit()
  {
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
