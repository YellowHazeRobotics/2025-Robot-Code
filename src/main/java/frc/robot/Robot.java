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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
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

  SparkMax coralGrabber;
  SparkMax intake;
  private static final int kJoystickPort = 0;
  private Joystick joystick;

  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;


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

    // Elevator
    elevatorLeader = new SparkMax (15, MotorType.kBrushless);
    elevatorFollower = new SparkMax (16, MotorType.kBrushless);
    elevatorLeaderConfig.inverted(true);
    elevatorFollowerConfig.follow(elevatorLeader, true);
    elevatorLeader.configure(elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    // Chopstick Lifter
    chopStickLifter = new SparkMax(17, MotorType.kBrushless);
    chopStickLifterConfig.inverted(false);
    chopStickLifterConfig.idleMode(IdleMode.kBrake);
    chopStickLifter.configure(chopStickLifterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Chopstick Pinser
    chopStickLeader = new SparkMax(18, MotorType.kBrushless);
    chopStickFollower = new SparkMax(19, MotorType.kBrushless);
    //chopStickLeaderConfig.inverted(false);
    chopStickFollowerConfig.follow(chopStickLeader, true);
    chopStickLeader.configure(chopStickLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    chopStickFollower.configure(chopStickFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Coral Arm
    //coralArm = new SparkMax(20, MotorType.kBrushless);
    
    coralGrabber = new SparkMax(20, MotorType.kBrushless);
    coralGrabber.configure(coralGrabberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //intake = new SparkMax(21, MotorType.kBrushless);
    //intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    joystick = new Joystick(0);
    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    closedLoopController = elevatorLeader.getClosedLoopController();
    encoder = elevatorLeader.getEncoder();
    

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */


    motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    //motorConfig.inverted(true);
    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.001)
        .i(0)
        .d(0.02)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.00001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    elevatorLeader.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorFollowerConfig.follow(elevatorLeader);
    elevatorFollowerConfig.idleMode(IdleMode.kBrake);
    elevatorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
        // Display encoder position and velocity
        SmartDashboard.putNumber("Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());
    
        if (SmartDashboard.getBoolean("Reset Encoder", false)) {
          SmartDashboard.putBoolean("Reset Encoder", false);
          // Reset the encoder position to 0
          encoder.setPosition(0);
        }
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
    encoder.setPosition(0);
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
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    // Elevator Leader                  
    if (joystick.getRawButtonPressed(1)) {
      elevatorLeader.set(0.2);
    }
    if (joystick.getRawButtonReleased(1)) {
      elevatorLeader.set(0); // When released the intake turns off
    }

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
    }*/
    
    // Chopstick Pincers: Left Bumper   
    if (joystick.getRawButtonPressed(5)) {
      chopStickLeader.set(1.0);
    }
    if (joystick.getRawButtonReleased(5)) {
      chopStickLeader.set(0);
    }

    // Chopstick unpincer: Left Trigger 
    if (joystick.getRawButtonPressed(3)) {
      chopStickLeaderConfig.inverted(true);
      chopStickLeader.configure(chopStickLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      chopStickLeader.set(1.0);
    }
    if (joystick.getRawButtonReleased(3)) {
      chopStickLeaderConfig.inverted(false);
      chopStickLeader.configure(chopStickLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      chopStickLeader.set(0);
    }
    
    // Chopstick Lifter: Right Trigger 
    if (joystick.getRawButtonPressed(6)) {
      chopStickLifter.set(0.2);
    }
    if (joystick.getRawButtonReleased(6)) {
      chopStickLifter.set(0);
    }

    if (joystick.getRawButtonPressed(10)) {
      chopStickLifterConfig.inverted(true);
      chopStickLifter.configure(chopStickLifterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      chopStickLifter.set(0.5);
    }
    if (joystick.getRawButtonReleased(10)) {
      chopStickLifterConfig.inverted(false);
      chopStickLifter.configure(chopStickLifterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      chopStickLifter.set(0);
    }


    if (joystick.getRawButtonPressed(8)) {
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
    }

    /*if (joystick.getRawButtonPressed(9)) {
      intake.set(0.5);
    }
    if (joystick.getRawButtonReleased(9)) {
      intake.set(0);
    }*/

    if (SmartDashboard.getBoolean("Control Mode", false)) {
      /*
       * Get the target velocity from SmartDashboard and set it as the setpoint
       * for the closed loop controller.
       */
      double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
      closedLoopController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    } else {
      /*
       * Get the target position from SmartDashboard and set it as the setpoint
       * for the closed loop controller.
       */
      double targetPosition = SmartDashboard.getNumber("Target Position", 0);
      closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

     // Define button
    boolean goToPosition = joystick.getRawButtonPressed(4); // Pressing 'A' moves elevator

    if (goToPosition) {
          double targetPosition = -35; // Set your desired position in encoder units
          closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

    /*if (joystick.getRawButtonPressed(10)) {
      intakeConfig.inverted(false);
      intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      intake.set(0.5);
    }
    if (joystick.getRawButtonReleased(10)) {
      intakeConfig.inverted(true);
      intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      intake.set(0);
    }*/
    // Coral Arm: B Button
    /*if (joystick.getRawButtonPressed(2)) {
      coralArm.set(1);
    }
    if (joystick.getRawButtonReleased(2)) {
      coralArm.set(0);
    }*/

    // Reference: ctrlaltftc.com/the-pid-controller
    
    RelativeEncoder encoder = elevatorLeader.getEncoder();
    if (joystick.getRawButtonPressed(4)) {
      while (encoder.getPosition() != Constants.ELEVATOR_POSITION) {

        System.out.println("WORK IN PROGRESS");
      }
    }
    
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
