// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
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
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
                                                                                
  private final SendableChooser<Command> autoChooser;

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();

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

    /*NamedCommands.registerCommand("Align Left Test", Commands.deferredProxy(() -> {
      int aprilTag = drivebase.getReefTargetTagID();
      return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
      }));*/

    NamedCommands.registerCommand("Shoot", coralSubsystem.manualForward().withTimeout(1));


    NamedCommands.registerCommand("Elevator to L1", elevatorSubsystem.moveToPosition(ElevatorConstants.L1HEIGHT));
    NamedCommands.registerCommand("Elevator to L2", elevatorSubsystem.moveToPosition(ElevatorConstants.L2HEIGHT));
    NamedCommands.registerCommand("Elevator to L3", elevatorSubsystem.moveToPosition(ElevatorConstants.L3HEIGHT));
    NamedCommands.registerCommand("Elevator to L4", elevatorSubsystem.moveToPosition(ElevatorConstants.L4HEIGHT));

    /*NamedCommands.registerCommand("Align Left", Commands.sequence(Commands.deferredProxy(() -> {
      int aprilTag = drivebase.getReefTargetTagID();
      return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
      }), Commands.deferredProxy(() -> {
        int aprilTag = drivebase.getReefTargetTagID();
        return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
        }),Commands.deferredProxy(() -> {
          int aprilTag = drivebase.getReefTargetTagID();
          return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
          }),Commands.deferredProxy(() -> {
            int aprilTag = drivebase.getReefTargetTagID();
            return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
            }),Commands.deferredProxy(() -> {
              int aprilTag = drivebase.getReefTargetTagID();
              return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
              }).asProxy()));*/


        /*NamedCommands.registerCommand("Align Left", Commands.sequence(Commands.deferredProxy(() -> {
          int aprilTag = drivebase.getReefTargetTagID();
          return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
          }), Commands.deferredProxy(() -> {
            int aprilTag = drivebase.getReefTargetTagID();
            return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
            }),Commands.deferredProxy(() -> {
              int aprilTag = drivebase.getReefTargetTagID();
              return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
              }),Commands.deferredProxy(() -> {
                int aprilTag = drivebase.getReefTargetTagID();
                return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
                }),Commands.deferredProxy(() -> {
                  int aprilTag = drivebase.getReefTargetTagID();
                  return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
                  }).withTimeout(6)));*/
              
    NamedCommands.registerCommand("Shoot L4", Commands.sequence(elevatorSubsystem.moveToPosition(10), coralSubsystem.manualForward().withTimeout(1), elevatorSubsystem.moveToPosition(ElevatorConstants.L1HEIGHT)));
    NamedCommands.registerCommand("Intake", coralSubsystem.manualBackWard().withTimeout(5));

    NamedCommands.registerCommand("Align Left Test", Commands.run(() -> {drivebase.alignToReefScore(() -> drivebase.getReefTargetTagID(), TargetSide.LEFT).schedule();}).withTimeout(5.0));

    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    SmartDashboard.putData("Auto Chooser", autoChooser);
    PathfindingCommand.warmupCommand().schedule();
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

    /*Command scoreTwoCoralCenter = Commands.sequence(Commands.deferredProxy(() -> {
              int aprilTag = drivebase.getReefTargetTagID();
              return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
              }),elevatorSubsystem.moveToPosition(10), 
              coralSubsystem.manualForward().withTimeout(1), 
              elevatorSubsystem.moveToPosition(ElevatorConstants.L1HEIGHT));*/
    
              
          
              
            /* AutoBuilder.buildAuto("Score 1st Coral", Commands.deferredProxy(() -> {
                int aprilTag = drivebase.getReefTargetTagID();
                return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
                }).withTimeout(4),AutoBuilder.buildAuto("Score 2nd Coral"));*/

    Command AlignLeft = Commands.deferredProxy(() -> {
      int aprilTag = drivebase.getReefTargetTagID();
      return drivebase.alignToReefScore(TargetSide.LEFT, aprilTag);});

    Command firstCoral = AutoBuilder.buildAuto("Score 1st Coral");
    Command secondCoral = AutoBuilder.buildAuto("Score 2nd Coral");


    /*Command scoreTwoCoralCenter = Commands.sequence(Commands.deferredProxy(() -> {
      int aprilTag = drivebase.getReefTargetTagID();
      return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
      }), AutoBuilder.buildAuto("Score 1st Coral"),Commands.deferredProxy(() -> {
                        int aprilTag = drivebase.getReefTargetTagID();
                        return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
                        }),AutoBuilder.buildAuto("Score 2nd Coral"));*/

    Command scoreTwoCoralCenter = Commands.sequence(AlignLeft, firstCoral, secondCoral);

    autoChooser.addOption("Score Two From Middle", scoreTwoCoralCenter);

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
      //ZERO GYRO
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

      //CONTOLS FOR ELEVATOR------------------------------
      //L4
      operatorXbox.y().onTrue(elevatorSubsystem.moveToPosition(ElevatorConstants.L4HEIGHT  + elevatorSubsystem.getL4_Increase()));
      //L3
      operatorXbox.b().onTrue(elevatorSubsystem.moveToPosition(ElevatorConstants.L3HEIGHT  + elevatorSubsystem.getL3_Increase()));
      //L2
      operatorXbox.a().onTrue(elevatorSubsystem.moveToPosition(ElevatorConstants.L2HEIGHT  + elevatorSubsystem.getL2_Increase()));
      //L1
      operatorXbox.x().onTrue(elevatorSubsystem.moveToPosition(ElevatorConstants.L1HEIGHT + elevatorSubsystem.getL1_Increase()));

      //Manual Movement
      operatorXbox.start().whileTrue(elevatorSubsystem.manualForward());
      operatorXbox.back().whileTrue(elevatorSubsystem.manualBackWard());

      //CONTROLS FOR ALGAE---------------------
      operatorXbox.povRight().whileTrue(algaeSubsystem.manualForward());
      operatorXbox.povLeft().whileTrue(algaeSubsystem.manualBackWard());

      //CONTROLS FOR CORAL---------------------
      operatorXbox.povUp().whileTrue(coralSubsystem.coralIntake());
      operatorXbox.povDown().whileTrue(coralSubsystem.manualForward());

      //CONTROLS FOR ALIGNMENT-----------------
      driverXbox.leftBumper().whileTrue(Commands.deferredProxy(() -> {
        int aprilTag = drivebase.getReefTargetTagID();
        return drivebase.alignToReefScore(aprilTag, TargetSide.LEFT);
        }));

       driverXbox.rightBumper().whileTrue(Commands.deferredProxy(() -> {
        int aprilTag = drivebase.getReefTargetTagID();
        return drivebase.alignToReefScore(aprilTag, TargetSide.RIGHT);
       }));

       driverXbox.povUp().whileTrue(Commands.deferredProxy(() -> {
        int aprilTag = drivebase.getReefTargetTagID();
        return drivebase.alignToAlgae(aprilTag);
       }));

       //CONTROLS FOR PID CHANGE-------------------
       driverXbox.y().onTrue(elevatorSubsystem.L4_Increase());
       driverXbox.b().onTrue(elevatorSubsystem.L3_Increase());
       driverXbox.a().onTrue(elevatorSubsystem.L2_Increase());
       driverXbox.x().onTrue(elevatorSubsystem.L1_Increase());

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
