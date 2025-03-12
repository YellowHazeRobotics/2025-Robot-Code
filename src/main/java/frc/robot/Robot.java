// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  private final PIDController pid = new PIDController(0.1, 0.1, 0);

  SparkMax elevatorLeader;
  SparkMax elevatorFollower;
  SparkMaxConfig elevatorLeaderConfig = new SparkMaxConfig();
  SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();

  SparkMax chopStickLifter;

  SparkMax chopStickLeader;
  SparkMax chopStickFollower;

  SparkMax chopStickPinserLeader;
  SparkMax chopStickPinserFollower;
  SparkMaxConfig chopStickLeaderConfig = new SparkMaxConfig();
  SparkMaxConfig chopStickFollowerConfig = new SparkMaxConfig();
  SparkMaxConfig chopStickLifterConfig = new SparkMaxConfig();
  SparkMaxConfig coralGrabberConfig = new SparkMaxConfig();
  SparkMaxConfig intakeConfig = new SparkMaxConfig();
  SparkMax coralArm;

  SparkMax coralManipulator;
  SparkMaxConfig coralManipulatorConfig = new SparkMaxConfig();

  SparkMax coralGrabber;

  SparkMax coralOutput;

  private static final int kJoystickPort = 0;
  private Joystick joystick;
  private Joystick operatorJoystick;

  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private SparkClosedLoopController pidController;

  private RelativeEncoder relativeEncoder;

  private double targetPosition;
  private boolean aPreviousState = false;
  private boolean bPreviousState = false;
  private boolean started = false;


  private SparkMaxConfig leaderConfig = new SparkMaxConfig();
  private SparkMaxConfig followerConfig = new SparkMaxConfig();


  //DECLARATIONS FOR CHOPSTICK LIFTER------------------------
  private SparkClosedLoopController chopStickLifterLoopController;
  private RelativeEncoder chopStickLifterEncoder;
  private double chopStickLifterTargetPosition;
  private double chopStickLifterSuckedInPosition;
  private boolean chopStickLifterStarted = false;
  private boolean chopStickLifterSuckedIn = false;

  //DECLARATIONS FOR CORAL MANIPULATOR---------------
  private SparkClosedLoopController coralManipulatorLoopController;
  private RelativeEncoder coralManipulatorEncoder;
  private double coralManipulatorShootingPosition;
  private double coralManipulatorIntakePosition;
  private boolean coralManipulatorShootingStarted = false;
  private boolean coralManipulatorIntakeStarted = false;

  //DECLARATIONS FOR ELEVATOR------------------------
  private SparkClosedLoopController elevatorLoopController;
  private RelativeEncoder elevatorEncoder;
  private double elevatorTargetPosition_L1;
  private double elevatorTargetPosition_L2;
  private double elevatorTargetPosition_L3;
  private double elevatorTargetPosition_L4;
  private boolean elevatorL1Started = false;
  private boolean elevatorL2Started = false;
  private boolean elevatorL3Started = false;
  private boolean elevatorL4Started = false;

  //DECLARATIONS FOR CORAL INTAKE--------------------
  SparkMax coralIntake;
  SparkMaxConfig coralIntakeConfig = new SparkMaxConfig();


  /*//DECLARATIONS FOR CORAL GRABBER---------------------------------
  private SparkClosedLoopController coralGrabberLoopController;
  private RelativeEncoder coralGrabberEncoder;
  private double coralGrabberTargetPosition;
  private boolean coralGrabberStarted = false;

  //Coral Input & Output---------------------------------
  private SparkClosedLoopController coralIntakeController;
  private RelativeEncoder coralIntakeEncoder;
  private double coralIntakeTargetPosition;
  private boolean coralIntakeStarted = false;

  private SparkClosedLoopController coralOutputController;
  private RelativeEncoder coralOutputEncoder;
  private double coralOutputTargetPosition;
  private boolean coralOutputStarted = false;*/

  public Robot()
  {
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
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    //SPARKMAX CONTROLS FOR NEOs

    // Chopstick Pinser
    chopStickLeader = new SparkMax(18, MotorType.kBrushless);
    chopStickFollower = new SparkMax(19, MotorType.kBrushless);
    //chopStickLeaderConfig.inverted(false);
    chopStickFollowerConfig.follow(chopStickLeader, true);
    chopStickLeader.configure(chopStickLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    chopStickFollower.configure(chopStickFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    /*
    // Coral Intake
    coralIntake = new SparkMax(21, MotorType.kBrushless);

    // Coral Output
    coralOutput = new SparkMax(22, MotorType.kBrushless);*/

    // Coral Arm
    //coralArm = new SparkMax(20, MotorType.kBrushless);

    //intake = new SparkMax(21, MotorType.kBrushless);
    //intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    joystick = new Joystick(0);
    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }


    //PID STUFF FOR CHOPSTICK LIFTER-----------------------
    chopStickLifter = new SparkMax(17, MotorType.kBrushless);
    chopStickLifterLoopController = chopStickLifter.getClosedLoopController();
    chopStickLifterEncoder = chopStickLifter.getEncoder();
    chopStickLifterConfig.inverted(false);
    chopStickLifterConfig.idleMode(IdleMode.kBrake);

    chopStickLifterConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    chopStickLifterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0.015)
        .outputRange(-1, 1);

    chopStickLifter.configure(chopStickLifterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //PID STUFF FOR ELEVATOR-----------------------
    elevatorLeader = new SparkMax (15, MotorType.kBrushless);
    elevatorFollower = new SparkMax (16, MotorType.kBrushless);
    elevatorLeaderConfig.inverted(true);
    elevatorFollowerConfig.follow(elevatorLeader, true);
    elevatorLoopController = elevatorLeader.getClosedLoopController();
    elevatorEncoder = elevatorLeader.getEncoder();

    elevatorLeaderConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    elevatorLeaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0.01)
        .outputRange(-1, 1);

    elevatorLeader.configure(elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //PID STUFF FOR CORAL MANIPULATOR-----------------------
    coralManipulator = new SparkMax(20, MotorType.kBrushless);
    coralManipulatorLoopController = coralManipulator.getClosedLoopController();
    coralManipulatorEncoder = coralManipulator.getEncoder();
    coralManipulatorConfig.inverted(false);
    coralManipulatorConfig.idleMode(IdleMode.kBrake);

    coralManipulatorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    coralManipulatorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.04)
        .i(0)
        .d(0.01)
        .outputRange(-1, 1);

    coralManipulator.configure(coralManipulatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    //CORAL INTAKE SET UP
    coralIntake = new SparkMax(22, MotorType.kBrushless);
    coralIntakeConfig.inverted(false);
    coralIntake.configure(coralIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  //PID STUFF FOR CORAL-----------------------
    /*coralGrabber = new SparkMax(20, MotorType.kBrushless);
    coralGrabberLoopController = coralGrabber.getClosedLoopController();
    coralGrabberEncoder = coralGrabber.getEncoder();
    coralGrabberConfig.inverted(false);
    coralGrabberConfig.idleMode(IdleMode.kBrake);

    coralGrabberConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    coralGrabberConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0.01)
        .outputRange(-1, 1);

    coralGrabber.configure(coralGrabberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      */
  }
   
  @Override
  public void robotPeriodic()
  {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Chopstick Lifter Actual Position", chopStickLifterEncoder.getPosition());
    SmartDashboard.putNumber("Chopstick Lifter Actual Velocity", chopStickLifterEncoder.getVelocity());

    SmartDashboard.putNumber("Elevator Actual Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Actual Velocity", elevatorEncoder.getVelocity());

    SmartDashboard.putNumber("CM Actual Position", coralManipulatorEncoder.getPosition());
    SmartDashboard.putNumber("CM Actual Velocity", coralManipulatorEncoder.getVelocity());

    /*SmartDashboard.putNumber("Coral Grabber Actual Position", coralGrabberEncoder.getPosition());
    SmartDashboard.putNumber("Coral Grabber Actual Velocity", coralGrabberEncoder.getVelocity());*/
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }

    chopStickLifterStarted = false;
    chopStickLifterSuckedIn = false;
    chopStickLifterTargetPosition = 1.6;
    chopStickLifterSuckedInPosition = 3.5999971389;
    chopStickLifterEncoder.setPosition(0);

    coralManipulatorShootingStarted = false;
    coralManipulatorIntakeStarted = false;
    coralManipulatorEncoder.setPosition(0);
    coralManipulatorIntakePosition = 2;
    coralManipulatorShootingPosition = 74;

    elevatorEncoder.setPosition(0);
    elevatorL1Started = false;
    elevatorL2Started = false;
    elevatorL3Started = false;
    elevatorL4Started = false;
    elevatorTargetPosition_L1 = 0;
    elevatorTargetPosition_L2 = 0;
    elevatorTargetPosition_L3 = 136.78370666;
    elevatorTargetPosition_L4 = 285.288;
  }

  @Override
  public void teleopPeriodic()
  {
    //CORAL MANIPULATOR INTAKE POSITION----------------------------------
    if(joystick.getRawButtonPressed(6) || coralManipulatorIntakeStarted) {
      coralManipulatorShootingStarted = false;
      coralManipulatorIntakeStarted = true;
      coralManipulatorLoopController.setReference(coralManipulatorIntakePosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } 

    //CORAL MANIPULATOR SHOOTING POSITION-------------------------------------
    if(joystick.getRawButtonPressed(5) || coralManipulatorShootingStarted) {
      coralManipulatorIntakeStarted = false;
      coralManipulatorShootingStarted = true;
      coralManipulatorLoopController.setReference(coralManipulatorShootingPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } 

    //ELEVATOR L1 SHOOTING POSITION--------------------------------------------
    if(joystick.getRawButtonPressed(3) || elevatorL1Started) {
      elevatorL2Started = false;
      elevatorL3Started = false;
      elevatorL4Started = false;
      elevatorLoopController.setReference(elevatorTargetPosition_L1, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } 

    //ELEVATOR L2 SHOOTING POSITION--------------------------------------------
    if(joystick.getRawButtonPressed(1) || elevatorL2Started) {
      elevatorL1Started = false;
      elevatorL3Started = false;
      elevatorL4Started = false;
      elevatorLoopController.setReference(elevatorTargetPosition_L2, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } 

    //ELEVATOR L1 SHOOTING POSITION--------------------------------------------
    if(joystick.getRawButtonPressed(2) || elevatorL3Started) {
      elevatorL2Started = false;
      elevatorL1Started = false;
      elevatorL4Started = false;
      elevatorLoopController.setReference(elevatorTargetPosition_L3, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } 

    //ELEVATOR L1 SHOOTING POSITION--------------------------------------------
    if(joystick.getRawButtonPressed(4) || elevatorL4Started) {
      elevatorL2Started = false;
      elevatorL3Started = false;
      elevatorL1Started = false;
      elevatorLoopController.setReference(elevatorTargetPosition_L4, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } 

    //CHOPSTICK LIFTER POSITION----------------
    if(joystick.getRawButtonPressed(8) || chopStickLifterStarted) {
      chopStickLifterSuckedIn = false;
      chopStickLifterStarted = true;
      chopStickLifterLoopController.setReference(chopStickLifterTargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } 
    if(joystick.getRawButtonPressed(7) || chopStickLifterSuckedIn) {
      chopStickLifterStarted = false;
      chopStickLifterSuckedIn = true;
      chopStickLifterLoopController.setReference(chopStickLifterSuckedInPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }  

    //CORAL OUTTAKE
    if(joystick.getPOV() == 0) {
      coralIntake.set(1);
    } 
    if (joystick.getPOV() != 0 && joystick.getPOV() != 180 && joystick.getPOV() != 90 && joystick.getPOV() != 270) {
      coralIntake.set(0);
    }

    //CORAL INTAKE
    if(joystick.getPOV() == 180) {
      coralIntakeConfig.inverted(true);
      coralIntake.configure(coralIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      coralIntake.set(1);
    } 
    if (joystick.getPOV() != 180 && joystick.getPOV() != 0 && joystick.getPOV() != 90 && joystick.getPOV() != 270) {
      coralIntakeConfig.inverted(false);
      coralIntake.configure(coralIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      coralIntake.set(0);
    }

    //CHOPSTICK OUTTAKE
    if (joystick.getPOV() == 270) {
      chopStickLeaderConfig.inverted(true);
      chopStickLeader.configure(chopStickLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      chopStickLeader.set(0.5);
    }

    if (joystick.getPOV() != 180 && joystick.getPOV() != 0 && joystick.getPOV() != 90 && joystick.getPOV() != 270) {
      chopStickLeaderConfig.inverted(false);
      chopStickLeader.configure(chopStickLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      chopStickLeader.set(0);
    }

    if(joystick.getPOV() == 90) {
      chopStickLeader.set(0.5);
    }

    if (joystick.getRawButtonPressed(9)) {
      elevatorLeader.set(0.5);
    }
    if (joystick.getRawButtonReleased(9)) {
      elevatorLeader.set(0); // When released the intake turns off
    }

    if (joystick.getRawButtonPressed(10)) {
      elevatorLeaderConfig.inverted(false);
      elevatorLeader.configure(elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      elevatorLeader.set(0.5);
    }
    if (joystick.getRawButtonReleased(10)) {
      elevatorLeaderConfig.inverted(true);
      elevatorLeader.configure(elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      elevatorLeader.set(0); // When released the intake turns off
    }

    

    SmartDashboard.putNumber("CM Target Position", coralManipulatorIntakePosition);
    SmartDashboard.putNumber("Current CM Position", coralManipulatorEncoder.getPosition());
    SmartDashboard.putBoolean("CM Started Running", coralManipulatorIntakeStarted);
    SmartDashboard.putNumber("CM Motor Output", coralManipulator.get());

    SmartDashboard.putNumber("Current Elevator Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Motor Output", elevatorLeader.get());

    SmartDashboard.putNumber("Current CL Position", chopStickLifterEncoder.getPosition());


    /* Elevator Leader                  
    if (joystick.getRawButtonPressed(7)) {
      elevatorLeaderConfig.inverted(false);
      elevatorLeader.configure(elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      elevatorLeader.set(0.5);
    }
    if (joystick.getRawButtonReleased(7)) {
      elevatorLeaderConfig.inverted(true);
      elevatorLeader.configure(elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      elevatorLeader.set(0); // When released the intake turns off
    }

    if (joystick.getRawButtonPressed(8)) {
      elevatorLeader.set(0.5);
    }
    if (joystick.getRawButtonReleased(8)) {
      elevatorLeader.set(0); // When released the intake turns off
    }
    */

    // Chopstick Ball Intake: Left Trigger
    /*if (joystick.getRawButtonPressed(Constants.L_Trigger)) {
      chopStickLeader.set(0.27);
    }
    if (joystick.getRawButtonReleased(5)) {
      chopStickLeader.set(0);
    }*/
    

    // Chopstick Ball Output: Right Trigger
    /*if (joystick.getRawButtonPressed(Constants.R_Trigger)) {
      chopStickLeaderConfig.inverted(true);
      chopStickLeader.configure(chopStickLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      chopStickLeader.set(1.0);
    }
    if (joystick.getRawButtonReleased(6)) {
      chopStickLeaderConfig.inverted(false);
      chopStickLeader.configure(chopStickLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      chopStickLeader.set(0);
    }

    /*if(joystick.getRawButtonPressed(3) || started) {
      started = true;
      chopStickLifterLoopController.setReference(chopStickLifterTargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } 
    
    /*SmartDashboard.putNumber("CL Target Position", chopStickLifterTargetPosition);
    SmartDashboard.putNumber("Current CL Position", chopStickLifterEncoder.getPosition());
    SmartDashboard.putBoolean("CL Started Running", started);
    SmartDashboard.putNumber("CL Motor Output", chopStickLifter.get());

    if(true  || coralIntakeStarted) {
      coralIntakeStarted = true;
      coralIntakeController.setReference(coralIntakeTargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    if(true  || coralOutputStarted) {
      coralOutputStarted = true;
      coralOutputController.setReference(coralOutputTargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }*/

    // Buttons A, B, X, Y for L1, L2, L3, & L4 of Elevator
   /* if(joystick.getRawButtonPressed(Constants.Button_X) || elevatorStarted) {
      elevatorStarted = true;
      elevatorLoopController.setReference(elevatorTargetPosition_L1, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    if(joystick.getRawButtonPressed(Constants.Button_A) || elevatorStarted) {
      elevatorStarted = true;
      elevatorLoopController.setReference(elevatorTargetPosition_L2, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    if(joystick.getRawButtonPressed(Constants.Button_B) || elevatorStarted) {
      elevatorStarted = true;
      elevatorLoopController.setReference(elevatorTargetPosition_L3, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    if(joystick.getRawButtonPressed(Constants.Button_Y) || elevatorStarted) {
      elevatorStarted = true;
      elevatorLoopController.setReference(elevatorTargetPosition_L4, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    if(joystick.getRawButtonReleased(1)) {
      elevatorStarted = false;
    }*/
    
    /*SmartDashboard.putNumber("Elevator Target Position", elevatorTargetPosition_L1);
    SmartDashboard.putNumber("Elevator Target Position", elevatorTargetPosition_L2);
    SmartDashboard.putNumber("Elevator Target Position", elevatorTargetPosition_L3);
    SmartDashboard.putNumber("Elevator Target Position", elevatorTargetPosition_L4);

    SmartDashboard.putNumber("Current Elevator Position", elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("Elevator Started Running", elevatorStarted);
    SmartDashboard.putNumber("Elevator Motor Output", elevatorLeader.get());*/


    /*if(joystick.getRawButtonPressed(2) || coralGrabberStarted) {
      coralGrabberStarted = true;
      coralGrabberLoopController.setReference(coralGrabberTargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } 
    if(joystick.getRawButtonReleased(2)) {
      coralGrabberStarted = false;
    }*/
    
    /*SmartDashboard.putNumber("Coral Grabber Target Position", coralGrabberTargetPosition);
    SmartDashboard.putNumber("Current Coral Grabber Position", coralGrabberEncoder.getPosition());
    SmartDashboard.putBoolean("Coral Grabber Started Running", coralGrabberStarted);
    SmartDashboard.putNumber("Coral Grabber Motor Output", coralGrabber.get());
    */
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
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

    // Chopstick Lifter: Right Trigger 
    /*if (joystick.getRawButtonPressed(2)) {
      chopStickLifter.set(0.2);
    }
    if (joystick.getRawButtonReleased(2)) {
      chopStickLifter.set(0);
    }*/

    /*if (joystick.getRawButtonPressed(6)) {
      chopStickLifterConfig.inverted(true);
      chopStickLifter.configure(chopStickLifterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      chopStickLifter.set(0.5);
    }
    if (joystick.getRawButtonReleased(6)) {
      chopStickLifterConfig.inverted(false);
      chopStickLifter.configure(chopStickLifterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      chopStickLifter.set(0);
    }*/


    /*if (joystick.getRawButtonPressed(8)) {
      coralGrabber.set(0.5);
    }
    if (joystick.getRawButtonReleased(8)) {
      coralGrabber.set(0);
    }

    if (joystick.getRawButtonPressed(7)) {
      coralGrabberConfig.inverted(false);
      coralGrabber.configure(coralGrabberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      coralGrabber.set(0.5);
    }
    if (joystick.getRawButtonReleased(7)) {
      coralGrabberConfig.inverted(true);
      coralGrabber.configure(coralGrabberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      coralGrabber.set(0);
    }*/

    /*if (joystick.getRawButtonPressed(9)) {
      intake.set(0.5);
    }
    if (joystick.getRawButtonReleased(9)) {
      intake.set(0);
    }*/

        /*if (joystick.getRawButtonPressed(1)) {
      elevatorLeaderConfig.idleMode(IdleMode.kBrake);
      elevatorFollowerConfig.idleMode(IdleMode.kBrake);
      elevatorLeader.configure(elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      elevatorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    if (joystick.getRawButtonReleased(1)) {
      elevatorLeaderConfig.idleMode(IdleMode.kCoast);
      elevatorFollowerConfig.idleMode(IdleMode.kCoast);
      elevatorLeader.configure(elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      elevatorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // When released the intake turns off

      L4 Coral  -> Y +++
      L3 Coral  -> B +++
      L2 Coral  -> A +++
      L1 Coral  -> X +++
      Intake Coral -> Forward DPad +++
      Output Coral -> Backward DPad +++
      Coral Loading Position -> Right Back +++
      Coral Scoring Position -> Left Back +++

      Pinser Up   -> START +++
      Intake Ball -> Right Trigger
      Output Ball -> Left Trigger
      Climbing    -> Back Button

    }*/