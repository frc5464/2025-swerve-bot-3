
package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.BlinkinLEDController;
import frc.robot.utils.BlinkinLEDController.BlinkinPattern;
import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.VisionSubsystem;

public class Robot extends TimedRobot{
  private Command m_autonomousCommand;
  private BlinkinLEDController leds = new BlinkinLEDController();
  private SubsystemManager subsystemManager;

  public Robot(){}

  @Override
  public void robotInit() {
    // m_robotContainer = new RobotContainer();
    subsystemManager = new SubsystemManager();
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
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic(){}

  @Override
  public void teleopInit()
  {
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
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
