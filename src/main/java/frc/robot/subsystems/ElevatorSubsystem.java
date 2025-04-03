package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {
    // MOTORS
    private SparkMax leadMotor, followMotor;
    // ENCODER
    private RelativeEncoder encoder;
    // FEEDBACK CONTROLLER
    private SparkClosedLoopController feedbackController;
    // CONFIGS
    SparkMaxConfig leadConfig, followConfig;
    // CURRENT POSITION
    public ElevatorPosition currentTargetPosition;
    //VELOCITY, CURRENT
    double motorVelocity, motorCurrent;
    public double L1_extra = 0;
    public double L2_extra = 0;
    public double L3_extra = 0;
    public double L4_extra = 0;

    /** Constructs an elevator. */
    public ElevatorSubsystem() {
        // MOTOR CONTROLLERS
        leadMotor = new SparkMax(ElevatorConstants.kLeaderID, MotorType.kBrushless);
        followMotor = new SparkMax(ElevatorConstants.kFollowerID, MotorType.kBrushless);
        // ENCODER
        encoder = leadMotor.getEncoder();
        // PID CONTROLLER
        feedbackController = leadMotor.getClosedLoopController();
        // POSITION
        currentTargetPosition = ElevatorPosition.STORED;
        // CONFIGS
        leadConfig = new SparkMaxConfig();
        followConfig = new SparkMaxConfig();
        //Zero Encoder
        encoder.setPosition(0);
        // CONFIGURE MOTORS
        configureMotors();
    }

    /** Sets the configurations for each motor. */
    private void configureMotors() {
        // LEADER CONFIG
        leadConfig
            .inverted(ElevatorConstants.kInverted)
            .idleMode(ElevatorConstants.kIdleMode)
            .smartCurrentLimit(ElevatorConstants.kStallLimit, ElevatorConstants.kFreeLimit);
        leadConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kFF, ClosedLoopSlot.kSlot0)
            .outputRange(ElevatorConstants.kMinimumOutputLimit, ElevatorConstants.kMaximumOutputLimit, ClosedLoopSlot.kSlot0);
        leadConfig.softLimit
            .forwardSoftLimit(ElevatorConstants.kMaximumRotationLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(ElevatorConstants.kMinimumRotationLimit)
            .reverseSoftLimitEnabled(true);
        leadMotor.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // FOLLOWER CONFIG
        followConfig.apply(leadConfig);
        followConfig.follow(leadMotor, ElevatorConstants.kFollowerInverted);
        followMotor.configureAsync(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // COMMAND FACTORIES TO REACH ENUM HEIGHT
    public Command setRaw(double percent) {
        return runOnce(() -> {
            leadMotor.set(percent);
        });
    }

    /** Sets the target height of the elevator. 
     * @param ElevatorPosition
     * The taregt position: including state and height.
    */
    public Command setPosition(ElevatorPosition position) {
        return runOnce(() -> {
            currentTargetPosition = position;
            feedbackController.setReference(position.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        });
    }

    public Command moveToPosition(double position) {
        return run(() -> {
            feedbackController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        });
    }

    /** Sets the target height of the elevator using trapezoidal profiling. 
     * @param ElevatorPosition
     * The taregt position: including state and height.
    */
    public Command setPositionSmartMotion(ElevatorPosition position) {
        return runOnce(() -> {
            // CHANGES CURRENT TARGET TO POS
            currentTargetPosition = position;
            // SETS FEEDBACKCONTROLLER TO POS
            feedbackController.setReference(position.position, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
        });
    }

    /**Waits until elevator reaches position within Tolerance.
     * @param ElevatorPosition
     * Enum for elevator height options. 
     */
    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(() -> {
            // TEST FOR IF ELEVATORERROR IS IN TOLERANCE OF TARGETPOSITION
            return isAtSetpoint();
        });
    }
    
    public boolean isAtSetpoint() {
        return (getElevatorError() < ElevatorConstants.kTolerance);
    }

    private double getElevatorError() {
        return Math.abs(Math.abs(encoder.getPosition()) - Math.abs(currentTargetPosition.position));
    }

    /**Resets encoder to 0*/
    public Command resetEncoder() {
        return runOnce(() -> {
                encoder.setPosition(0);
            });
    }
    /**Ensures that motor is set to 0 after triggering bottomLimitSwitch*/
    public Command stopMotorCommand() {
        return runOnce(() -> {
            leadMotor.set(0);
        });
    }

    public Command manuallyRunForward() {
        return run(() -> {
            leadMotor.set(0.5);
        });
    }

    /** Stops the motor manually, ignoring all commands. */
    public void stopMotorManual() {
        leadMotor.set(0);
    }

    public Command printPosition() {
        return runOnce(() -> System.out.println(encoder.getPosition()));
    }

    public Command L1_Increase(){
        return runOnce(() -> L1_Increaser());
    }

    public void L1_Increaser(){
        L1_extra +=1 ;
    }

    public double getL1_Increase(){
        return L1_extra;
    }

    public Command L2_Increase(){
        return runOnce(() -> L2_Increaser());
    }

    public void L2_Increaser(){
        L2_extra +=1 ;
    }

    public double getL2_Increase(){
        return L2_extra;
    }

    public Command L3_Increase(){
        return runOnce(() -> L3_Increaser());
    }

    public void L3_Increaser(){
        L3_extra +=1 ;
    }

    public double getL3_Increase(){
        return L3_extra;
    }

    public Command L4_Increase(){
        return runOnce(() -> L4_Increaser());
    }

    public void L4_Increaser(){
        L4_extra +=1 ;
    }

    public double getL4_Increase(){
        return L4_extra;
    }


    public void setIdleMode(IdleMode idleMode) {
        leadConfig.idleMode(idleMode);
        leadMotor.configureAsync(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        followConfig.idleMode(idleMode);
        followMotor.configureAsync(followConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", encoder.getPosition());
        SmartDashboard.putNumber("Target Elevator Height", currentTargetPosition.getRotations());
        SmartDashboard.putBoolean("Elevator at Setpoint", isAtSetpoint());
    }
    
    public Command manualForward(){
        return startEnd(
          () -> leadMotor.set(0.5),
          () -> leadMotor.set(0));
      }
      
      public Command manualBackWard(){
        return startEnd(
          () -> leadMotor.set(-0.5), //Make negative????
          () -> leadMotor.set(0));
      }

    /** Enum for elevator height options. Contains heightCentimeters, which is the target height in centimeters. */
    public enum ElevatorPosition {
        // ENUMS FOR POSITIONS
        STORED(0), PRIME(-1), COBRA_STANCE(-1),
        PID_TESTING(20),

        ALGAE_INTAKE(-1), ALGAE_DESCORE_L_TWO(-1), ALGAE_DESCORE_L_THREE(-1),
      
        GROUND_INTAKE(7), CORAL_STATION_INTAKE(0),

        L_ONE(0), L_TWO(10), L_THREE(11), L_FOUR(47);

        private double position;
        /**Constrcutor for height for ElevatorPositions (Enum for Elevator poses)
        * @param position
        * verticle movement in centimeters
        */
        ElevatorPosition(double position) {
            this.position = position;
        }

        public double getRotations() {
            return this.position;
        }
    }
}