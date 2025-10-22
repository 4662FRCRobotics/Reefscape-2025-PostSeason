// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandConstants;

public class HandSubsystem extends SubsystemBase {
  /** Creates a new HandSubsystem. */
  private SparkMax m_motorHand;
  private SparkMaxConfig m_motorConfigHand;
  private RelativeEncoder m_encoderHand;
  private AbsoluteEncoder m_absEncoderHand;
  private SparkClosedLoopController m_closedLoopHand;
  private SoftLimitConfig m_SoftLimitHand;
  private double m_handTargetPosition = HandConstants.kHandDown;

  public HandSubsystem() {
    m_motorHand = new SparkMax(HandConstants.motorHand, MotorType.kBrushless);
      m_motorConfigHand = new SparkMaxConfig();
      m_motorConfigHand.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(HandConstants.kCurrentLimit)
        .secondaryCurrentLimit(HandConstants.kSecondaryCurrentLimit);
      m_motorConfigHand.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
      m_motorConfigHand.closedLoop.maxMotion
        .maxAcceleration(2500)
        .maxVelocity(5000)
        .allowedClosedLoopError(1);
      m_closedLoopHand = m_motorHand.getClosedLoopController();
      m_motorConfigHand.absoluteEncoder
        .inverted(true)
        .positionConversionFactor(360);
       m_SoftLimitHand = new SoftLimitConfig();
      m_SoftLimitHand.forwardSoftLimit(HandConstants.kFwdSoftLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(HandConstants.kRevSoftLimit)
        .reverseSoftLimitEnabled(true);
      m_motorConfigHand.apply(m_SoftLimitHand);
  
      m_motorHand.configure(m_motorConfigHand, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_encoderHand = m_motorHand.getEncoder();
      m_absEncoderHand = m_motorHand.getAbsoluteEncoder();

      setRelativeEncoder();
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber( "HandPostion", m_encoderHand.getPosition());
    SmartDashboard.putNumber("Hand Target", m_handTargetPosition);
    SmartDashboard.putNumber("Hand Abs Enc", m_absEncoderHand.getPosition());
    SmartDashboard.putNumber("Hand Current",m_motorHand.getOutputCurrent());
  }

  public boolean isHandStalled() {
   return m_motorHand.getOutputCurrent()>= HandConstants.kHandStallCurrent;
  } 

  public void stopHandMotor() {
    m_motorHand.set(0);
  }

  public Command cmdStopHand() {
    return Commands.runOnce(() -> stopHandMotor() , this);
  }

  private void adjustHandPosition(boolean isAdjustUp) {
    if (isAdjustUp) {
      m_handTargetPosition = m_handTargetPosition + HandConstants.kPostionAdjust;
    } else {
      m_handTargetPosition = m_handTargetPosition - HandConstants.kPostionAdjust;
    }
    //if (m_handTargetPosition > HandConstants.kFwdSoftLimit) {
    //  m_handTargetPosition = HandConstants.kFwdSoftLimit}
    // etc
    m_closedLoopHand.setReference(m_handTargetPosition, ControlType.kMAXMotionPositionControl);
  }

  public void setHandPosition() {
    if (m_handTargetPosition >= HandConstants.kHandDownish) {
      m_handTargetPosition = HandConstants.kHandUp;
    }
    else {
      m_handTargetPosition = HandConstants.kHandDown;
    }
    //m_handTargetPosition = targetPosition;
    setHandPosition(m_handTargetPosition);
  }

  private void setHandPosition(double targetPosition) {
    m_handTargetPosition = targetPosition;
    m_closedLoopHand.setReference(m_handTargetPosition, ControlType.kMAXMotionPositionControl);
  }

  public Command cmdSetHandUp() {
    return Commands.runOnce(() -> setHandPosition(HandConstants.kHandUp), this);
  }

  public Command cmdSetHandDown() {
    return Commands.runOnce(() -> setHandPosition(HandConstants.kHandDown), this);
  }

  public Command cmdAdjustHandPosition(boolean isAdjustUp) {
    return Commands.runOnce(() -> adjustHandPosition(isAdjustUp), this);
  }

  public Command cmdSetHandPosition() {
    return Commands.runOnce(() -> setHandPosition() , this);
  }

  public Command cmdHandScore() {
    return Commands.runOnce(() -> setHandPosition(HandConstants.kHandScore), this);
  }

  private void handZero(){
  stopHandMotor();
  m_encoderHand.setPosition(0);
  }

  public Command cmdHandZero(){
  //return Commands.runOnce(() -> stopElevatorMotor(), this)
   // .andThen(Commands.run(() -> runMotor(ElevatorConstants.kElevatorDown) , this))
   return Commands.run(() ->  m_closedLoopHand.setReference(HandConstants.kHandDown, ControlType.kMAXMotionPositionControl))
    .until(() -> m_motorHand.getOutputCurrent() >= HandConstants.kCurrentLimit)
    .andThen(() -> handZero() , this)
    ;
  }

  public boolean isHandDown() {
    return m_handTargetPosition >= (HandConstants.kHandDown) - 25;
  }

  public BooleanSupplier isHandDownSplr() {
    return () -> isHandDown();
  }

  private void setRelativeEncoder() {
    m_encoderHand.setPosition(-m_absEncoderHand.getPosition()/HandConstants.kEncoderDegrees);
  }
}