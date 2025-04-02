// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.ConfigurationFailedException;
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

public class CoralSubsystem extends SubsystemBase {
  
  private SparkMax m_Coral;
  SparkMaxConfig coralConfig;
  LaserCan laserCan = new LaserCan(19);
  Measurement laserMeasurement;
  private double currentTargetPosition;

  private RelativeEncoder encoder;
  private SparkClosedLoopController feedbackController;

  public CoralSubsystem() {
    m_Coral = new SparkMax(CoralConstants.kCoralMotorID, MotorType.kBrushless);

    coralConfig = new SparkMaxConfig();

    configureMotors();

    encoder = m_Coral.getEncoder();
    feedbackController = m_Coral.getClosedLoopController();
  }

  public void configureMotors(){
    coralConfig
    .inverted(CoralConstants.kInverted)
    .idleMode(CoralConstants.kIdleMode);

    m_Coral.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command spinCoralMotors(double position) {
    return runOnce(() -> {
      currentTargetPosition = position;
      feedbackController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    });
  }

  public Command resetEncoder() {
    return runOnce(() -> {
            encoder.setPosition(0);
        });
}

public void stopMotorManual() {
  m_Coral.set(0);
}

public Command manualForward(){
  return startEnd(
    () -> m_Coral.set(0.5),
    () -> m_Coral.set(0));
}

public Command manualBackWard(){
  return startEnd(
    () -> m_Coral.set(-0.5), //Make negative????
    () -> m_Coral.set(0));
}

public Command coralIntake(){
  return run(() -> m_Coral.set(0.2)).until(() -> isCoralInIntake()).finallyDo(() -> m_Coral.set(0));
}

public boolean isCoralInIntake() {
  Measurement measurement = laserCan.getMeasurement();
       
  return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm < 69;
}

public void normalInvert(){
  coralConfig.inverted(CoralConstants.kInverted);
  m_Coral.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

public void invertToBack(){
  coralConfig.inverted(!CoralConstants.kInverted);
  m_Coral.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Intake Position", encoder.getPosition());
    laserMeasurement = laserCan.getMeasurement();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
