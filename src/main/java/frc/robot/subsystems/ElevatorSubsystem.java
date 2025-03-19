package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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


public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax elevatorLeader = new SparkMax (15, MotorType.kBrushless);
    private SparkMax elevatorFollower = new SparkMax (16, MotorType.kBrushless);
    private SparkMaxConfig elevatorLeaderConfig = new SparkMaxConfig();
    private SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
    private SparkClosedLoopController elevatorLoopController = elevatorLeader.getClosedLoopController();
    private RelativeEncoder elevatorEncoder = elevatorLeader.getEncoder();
    private boolean elevatorStarted = false;

    public ElevatorSubsystem(){
        elevatorLeaderConfig.inverted(true);
        elevatorFollowerConfig.follow(elevatorLeader, true);

        elevatorLeaderConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

        elevatorLeaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.3)
        .i(0)
        .d(0.01)
        .outputRange(-1, 1);

        elevatorLeader.configure(elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorFollower.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
    }

    public void setMotors(double speed) {
        elevatorLeader.set(speed);
    }

    public void setElevator(double position) {
        elevatorLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    
}
