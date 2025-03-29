// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivebaseConstants.TargetSide;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
                                                                                
  private final SendableChooser<Command> autoChooser;

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  Trigger zeroTrigger;
  
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverXbox.getLeftY(),
                                                                () -> -driverXbox.getLeftX())
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
        //naming convention = alliance, clock position based on johan's diagram, side of position
    NamedCommands.registerCommand("Red Align 12 Left", drivebase.alignToReefScore(7,TargetSide.LEFT));
    NamedCommands.registerCommand("Red Align 12 Right", drivebase.alignToReefScore(7,TargetSide.RIGHT));
    NamedCommands.registerCommand("Red Align 2 Right", drivebase.alignToReefScore(6,TargetSide.RIGHT));
    NamedCommands.registerCommand("Red Align 2 Left", drivebase.alignToReefScore(6,TargetSide.LEFT));
    NamedCommands.registerCommand("Red Align 4 Right", drivebase.alignToReefScore(11,TargetSide.RIGHT));
    NamedCommands.registerCommand("Red Align 4 Left", drivebase.alignToReefScore(11,TargetSide.LEFT));
    NamedCommands.registerCommand("Red Align 6 Right", drivebase.alignToReefScore(10,TargetSide.RIGHT));
    NamedCommands.registerCommand("Red Align 6 Left", drivebase.alignToReefScore(10,TargetSide.LEFT));
    NamedCommands.registerCommand("Red Align 8 Right", drivebase.alignToReefScore(9,TargetSide.RIGHT));
    NamedCommands.registerCommand("Red Align 8 Left", drivebase.alignToReefScore(9,TargetSide.LEFT));
    NamedCommands.registerCommand("Red Align 10 Right", drivebase.alignToReefScore(8,TargetSide.RIGHT));
    NamedCommands.registerCommand("Red Align 10 Left", drivebase.alignToReefScore(8,TargetSide.LEFT));

    NamedCommands.registerCommand("Blue Align 12 Left", drivebase.alignToReefScore(18,TargetSide.LEFT));
    NamedCommands.registerCommand("Blue Align 12 Right", drivebase.alignToReefScore(18,TargetSide.RIGHT));
    NamedCommands.registerCommand("Blue Align 2 Right", drivebase.alignToReefScore(19,TargetSide.RIGHT));
    NamedCommands.registerCommand("Blue Align 2 Left", drivebase.alignToReefScore(19,TargetSide.LEFT));
    NamedCommands.registerCommand("Blue Align 4 Right", drivebase.alignToReefScore(20,TargetSide.RIGHT));
    NamedCommands.registerCommand("Blue Align 4 Left", drivebase.alignToReefScore(20,TargetSide.LEFT));
    NamedCommands.registerCommand("Blue Align 6 Right", drivebase.alignToReefScore(21,TargetSide.RIGHT));
    NamedCommands.registerCommand("Blue Align 6 Left", drivebase.alignToReefScore(21,TargetSide.LEFT));
    NamedCommands.registerCommand("Blue Align 8 Right", drivebase.alignToReefScore(22,TargetSide.RIGHT));
    NamedCommands.registerCommand("Blue Align 8 Left", drivebase.alignToReefScore(22,TargetSide.LEFT));
    NamedCommands.registerCommand("Blue Align 10 Right", drivebase.alignToReefScore(17,TargetSide.RIGHT));
    NamedCommands.registerCommand("Blue Align 10 Left", drivebase.alignToReefScore(17,TargetSide.LEFT));
    NamedCommands.registerCommand("align Left",Commands.run(()->{drivebase.alignToReefScore(()->drivebase.getReefTargetTagID(), TargetSide.LEFT).schedule();}));

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    RobotModeTriggers.autonomous().onTrue(elevatorSubsystem.resetEncoder());

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.b().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      /*driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );*/
      //driverXbox.start().whileTrue(Commands.none());
      //driverXbox.back().whileTrue(Commands.none());
      //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      //driverXbox.rightBumper().onTrue(Commands.none());

      /*driverXbox.x().whileTrue(elevatorSubsystem.manualForward());
      driverXbox.b().whileTrue(elevatorSubsystem.manualBackWard());

      driverXbox.y().onTrue(elevatorSubsystem.moveToPosition(80));
      driverXbox.a().onTrue(elevatorSubsystem.moveToPosition(0));*/


      driverXbox.leftBumper().onTrue(Commands.runOnce(()->{drivebase.alignToReefScore(()->drivebase.getReefTargetTagID(), TargetSide.LEFT).schedule();}));
      driverXbox.rightBumper().onTrue(Commands.runOnce(()->{drivebase.alignToReefScore(()->drivebase.getReefTargetTagID(), TargetSide.RIGHT).schedule();}));
     
      driverXbox.povUp().onTrue(Commands.runOnce(()->{drivebase.alignToAlgae(()->drivebase.getReefTargetTagID()).schedule();}));

      var allianceColor = DriverStation.getAlliance();

      if (allianceColor.isPresent() && allianceColor.get() == DriverStation.Alliance.Red) {
        driverXbox.x().and(driverXbox.leftBumper()).whileTrue(drivebase.alignToReefScore(6,TargetSide.LEFT));
        driverXbox.x().and(driverXbox.rightBumper()).whileTrue(drivebase.alignToReefScore(6,TargetSide.RIGHT));
    
        driverXbox.y().and(driverXbox.leftBumper()).whileTrue(drivebase.alignToReefScore(11,TargetSide.LEFT));
        driverXbox.y().and(driverXbox.rightBumper()).whileTrue(drivebase.alignToReefScore(11,TargetSide.RIGHT));
    
        driverXbox.b().and(driverXbox.leftBumper()).whileTrue(drivebase.alignToReefScore(9,TargetSide.LEFT));
        driverXbox.b().and(driverXbox.rightBumper()).whileTrue(drivebase.alignToReefScore(9,TargetSide.RIGHT));
    
        driverXbox.a().and(driverXbox.leftBumper()).whileTrue(drivebase.alignToReefScore(8,TargetSide.LEFT));
        driverXbox.a().and(driverXbox.rightBumper()).whileTrue(drivebase.alignToReefScore(8,TargetSide.RIGHT));
  
        driverXbox.b().and(driverXbox.y()).and(driverXbox.leftBumper()).whileTrue(drivebase.alignToReefScore(10, TargetSide.LEFT));
        driverXbox.b().and(driverXbox.y()).and(driverXbox.rightBumper()).whileTrue(drivebase.alignToReefScore(10, TargetSide.RIGHT));
  
        driverXbox.a().and(driverXbox.x()).and(driverXbox.leftBumper()).whileTrue(drivebase.alignToReefScore(7, TargetSide.LEFT));
        driverXbox.a().and(driverXbox.x()).and(driverXbox.rightBumper()).whileTrue(drivebase.alignToReefScore(7, TargetSide.RIGHT));
    
      } else if (allianceColor.isPresent() && allianceColor.get() == DriverStation.Alliance.Blue) {
        driverXbox.x().and(driverXbox.leftBumper()).whileTrue(drivebase.alignToReefScore(19,TargetSide.LEFT));
        driverXbox.x().and(driverXbox.rightBumper()).whileTrue(drivebase.alignToReefScore(19,TargetSide.RIGHT));
    
        driverXbox.y().and(driverXbox.leftBumper()).whileTrue(drivebase.alignToReefScore(20,TargetSide.LEFT));
        driverXbox.y().and(driverXbox.rightBumper()).whileTrue(drivebase.alignToReefScore(20,TargetSide.RIGHT));
    
        driverXbox.b().and(driverXbox.leftBumper()).whileTrue(drivebase.alignToReefScore(22,TargetSide.LEFT));
        driverXbox.b().and(driverXbox.rightBumper()).whileTrue(drivebase.alignToReefScore(22,TargetSide.RIGHT));
    
        driverXbox.a().and(driverXbox.leftBumper()).whileTrue(drivebase.alignToReefScore(17,TargetSide.LEFT));
        driverXbox.a().and(driverXbox.rightBumper()).whileTrue(drivebase.alignToReefScore(17,TargetSide.RIGHT));
  
        driverXbox.b().and(driverXbox.y()).and(driverXbox.leftBumper()).whileTrue(drivebase.alignToReefScore(21, TargetSide.LEFT));
        driverXbox.b().and(driverXbox.y()).and(driverXbox.rightBumper()).whileTrue(drivebase.alignToReefScore(21, TargetSide.RIGHT));
  
        driverXbox.a().and(driverXbox.x()).and(driverXbox.leftBumper()).whileTrue(drivebase.alignToReefScore(18, TargetSide.LEFT));
        driverXbox.a().and(driverXbox.x()).and(driverXbox.rightBumper()).whileTrue(drivebase.alignToReefScore(18, TargetSide.RIGHT));
      } else {
        System.out.println("no alliance found");
      }
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
