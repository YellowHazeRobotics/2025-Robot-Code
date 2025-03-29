// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

public class AlgaeSubsystem extends SubsystemBase {
  
  private SparkMax m_Algae;
  SparkMaxConfig algaeConfig;

  private double currentTargetPosition;

  private RelativeEncoder encoder;
  private SparkClosedLoopController feedbackController;

  public AlgaeSubsystem() {
    m_Algae = new SparkMax(AlgaeConstants.kAlgaeMotorID, MotorType.kBrushless);

    algaeConfig = new SparkMaxConfig();

    configureMotors();

    encoder = m_Algae.getEncoder();
    feedbackController = m_Algae.getClosedLoopController();
  }

  public void configureMotors(){
    algaeConfig
    .inverted(AlgaeConstants.kInverted)
    .idleMode(AlgaeConstants.kIdleMode);

    m_Algae.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command spinAlgaeMotors(double position) {
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
  m_Algae.set(0);
}

public Command manualForward(){
  return startEnd(
    () -> m_Algae.set(0.5),
    () -> m_Algae.set(0));
}

public Command manualBackWard(){
  return startEnd(
    () -> m_Algae.set(-0.5),
    () -> m_Algae.set(0));
}
@Override
public void periodic() {
    SmartDashboard.putNumber("Algae Intake Position", encoder.getPosition());
  }

@Override
public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
