// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import com.pathplanner.lib.auto.NamedCommands;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.OperatorConstants;
// import frc.robot.subsystems.SwerveSubsystem;
// import swervelib.SwerveInputStream;

// /**
//  * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
//  * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
//  * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
//  */
// public class RobotContainer
// {

//   // Replace with CommandPS4Controller or CommandJoystick if needed
//   final         CommandXboxController driverXbox = new CommandXboxController(0);
//   // The robot's subsystems and commands are defined here...
//   private final SwerveSubsystem       drivebase  = new SwerveSubsystem();

//   /**
//    * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
//    */
//   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
//                                                                 () -> driverXbox.getLeftY() * -1,
//                                                                 () -> driverXbox.getLeftX() * -1)
//                                                             .withControllerRotationAxis(driverXbox::getRightX)
//                                                             .deadband(OperatorConstants.DEADBAND)
//                                                             .scaleTranslation(0.8)
//                                                             .allianceRelativeControl(true);

//   /**
//    * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
//    */
//   SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
//                                                                                              driverXbox::getRightY)
//                                                            .headingWhile(true);

//   /**
//    * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
//    */
//   SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
//                                                              .allianceRelativeControl(false);

//   SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
//                                                                         () -> -driverXbox.getLeftY(),
//                                                                         () -> -driverXbox.getLeftX())
//                                                                     .withControllerRotationAxis(() -> driverXbox.getRawAxis(
//                                                                         2))
//                                                                     .deadband(OperatorConstants.DEADBAND)
//                                                                     .scaleTranslation(0.8)
//                                                                     .allianceRelativeControl(true);
//   // Derive the heading axis with math!
//   SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
//                                                                                .withControllerHeadingAxis(() ->
//                                                                                                               Math.sin(
//                                                                                                                   driverXbox.getRawAxis(
//                                                                                                                       2) *
//                                                                                                                   Math.PI) *
//                                                                                                               (Math.PI *
//                                                                                                                2),
//                                                                                                           () ->
//                                                                                                               Math.cos(
//                                                                                                                   driverXbox.getRawAxis(
//                                                                                                                       2) *
//                                                                                                                   Math.PI) *
//                                                                                                               (Math.PI *
//                                                                                                                2))
//                                                                                .headingWhile(true)
//                                                                                .translationHeadingOffset(true)
//                                                                                .translationHeadingOffset(Rotation2d.fromDegrees(
//                                                                                    0));

//   /**
//    * The container for the robot. Contains subsystems, OI devices, and commands.
//    */
//   public RobotContainer()
//   {
//     // Configure the trigger bindings
//     configureBindings();
//     DriverStation.silenceJoystickConnectionWarning(true);
//     NamedCommands.registerCommand("test", Commands.print("I EXIST"));
//   }

//   /**
//    * Use this method to define your trigger->command mappings. Triggers can be created via the
//    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
//    * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
//    * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
//    * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
//    */
//   private void configureBindings()
//   {
//     Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
//     Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

//     if (RobotBase.isSimulation())
//     {
//       drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
//     } else
//     {
//       drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
//     }

//       // driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
//       // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
//       // driverXbox.start().whileTrue(Commands.none());
//       // driverXbox.back().whileTrue(Commands.none());
//       // driverXbox.rightBumper().onTrue(Commands.none());

//   }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand()
//   {
//     // An example command will be run in autonomous
//     return drivebase.getAutonomousCommand("New Auto");
//   }

//   public void setMotorBrake(boolean brake)
//   {
//     drivebase.setMotorBrake(brake);
//   }
// }

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser); 
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}