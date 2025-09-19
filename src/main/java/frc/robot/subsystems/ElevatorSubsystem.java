// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Configs.ElevatorMotor;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax m_motorElevatorLeft;
  private SparkMax m_motorElevatorRight;
  private SparkMaxConfig m_motorConfigRight;
  private SparkClosedLoopController m_closedLoopElevatorLeft;
  private SparkMaxConfig m_motorConfigLeft;
  private double m_elevatorTargetPostion;
  private double m_outputCurrentLeft;
  private double m_outputCurrentRight;
  private RelativeEncoder m_encoderElevatorLeft;
  private double m_elevatorP;
  private double m_elevatorI;
  private double m_elevatorD;
  private SoftLimitConfig m_motorSoftLimitLeft;


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorP = 0.4;
    m_elevatorI = 0;
    m_elevatorD = 0;
    m_motorElevatorLeft = new SparkMax(ElevatorConstants.motorElevatorLeft, MotorType.kBrushless);
      m_motorConfigLeft = new SparkMaxConfig();
      m_motorConfigLeft.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
        .secondaryCurrentLimit(ElevatorConstants.kSecondaryCurrentLimit);
      m_motorConfigLeft.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(m_elevatorP)
        .i(m_elevatorI)
        .d(m_elevatorD)
        .outputRange(-1, 1);
      m_motorConfigLeft.closedLoop.maxMotion
        .maxAcceleration(2500)
        .maxVelocity(5000)
        .allowedClosedLoopError(1);
      m_motorSoftLimitLeft = new SoftLimitConfig();
      m_motorSoftLimitLeft.forwardSoftLimit(ElevatorConstants.kFwdSoftLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(ElevatorConstants.kRevSoftLimit)
        .reverseSoftLimitEnabled(true);
        
    m_closedLoopElevatorLeft = m_motorElevatorLeft.getClosedLoopController();
    m_motorElevatorLeft.configure(m_motorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_motorElevatorRight = new SparkMax(ElevatorConstants.motorElevatorRight, MotorType.kBrushless);
    m_motorConfigRight = new SparkMaxConfig();
    m_motorConfigRight
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
      .secondaryCurrentLimit(ElevatorConstants.kSecondaryCurrentLimit)
      .follow(ElevatorConstants.motorElevatorLeft,true);
    m_motorElevatorRight.configure(m_motorConfigRight,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_encoderElevatorLeft = m_motorElevatorLeft.getEncoder();

    m_elevatorTargetPostion = ElevatorConstants.kHandStartUpInches;
    m_outputCurrentLeft = 0.0;
    m_outputCurrentRight = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double outputCurrentLeft = m_motorElevatorLeft.getOutputCurrent();
    double outputCurrentRight = m_motorElevatorRight.getOutputCurrent();
    if (outputCurrentLeft > m_outputCurrentLeft) {
      m_outputCurrentLeft = outputCurrentLeft;
    }
    if (outputCurrentRight > m_outputCurrentRight) {
      m_outputCurrentRight = outputCurrentRight;
    }
    SmartDashboard.putNumber("Left Current",m_outputCurrentLeft);
    SmartDashboard.putNumber("Right Current",m_outputCurrentRight);
    SmartDashboard.putNumber("Elevator Position",m_elevatorTargetPostion);
    SmartDashboard.putNumber("Actual Pos", getPositionInches());
    SmartDashboard.putNumber("Elevator P" , m_elevatorP);
    SmartDashboard.putNumber( "Elevator I" , m_elevatorI);
    SmartDashboard.putNumber( "Elevator D" , m_elevatorD);
  }

  private void updateElevatorConfig() {
    m_elevatorP = SmartDashboard.getNumber("Elevator P" , m_elevatorP);
    m_elevatorI = SmartDashboard.getNumber("Elevator I" , m_elevatorI);
    m_elevatorD = SmartDashboard.getNumber("Elevator D" , m_elevatorD);
    m_motorConfigLeft.closedLoop
        //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(m_elevatorP)
        .i(m_elevatorI)
        .d(m_elevatorD)
        .outputRange(-1, 1);
  }

  public Command cmdUpdateElevatorConfig() {
    return Commands.runOnce(() -> updateElevatorConfig() , this).ignoringDisable(true);
  }

  public void setElevatorPosition(double targetPosition, BooleanSupplier isHandDown) {

    if (targetPosition < ElevatorConstants.kHandStartUpInches) {
      if (isHandDown.getAsBoolean()) {
        targetPosition = ElevatorConstants.kHandStartUpInches;
      }
    }
    m_elevatorTargetPostion = targetPosition;
    double targetEncoderPosition = ((m_elevatorTargetPostion - ElevatorConstants.kHandStartUpInches) / ElevatorConstants.kWinchCircumferenceInches) * ElevatorConstants.kGearRatio;
    m_closedLoopElevatorLeft.setReference(targetEncoderPosition, ControlType.kMAXMotionPositionControl);
  }

  private void adjustElevatorPosition(boolean isAdjustUp, BooleanSupplier isHandDown) {
    if (isAdjustUp) {
      m_elevatorTargetPostion = m_elevatorTargetPostion + ElevatorConstants.kPostionAdjust;
    } else {
      m_elevatorTargetPostion = m_elevatorTargetPostion - ElevatorConstants.kPostionAdjust;
    }
    setElevatorPosition(m_elevatorTargetPostion, isHandDown);
  //  m_closedLoopElevatorLeft.setReference(m_elevatorTargetPostion, ControlType.kMAXMotionPositionControl);
  }

  public Command cmdAdjustElevatorPosition(boolean isAdjustUp, BooleanSupplier isHandDown) {
    return Commands.runOnce(() -> adjustElevatorPosition(isAdjustUp, isHandDown) , this);
  }

  public Command cmdSetElevatorPosition(double targetPosition, BooleanSupplier isHandDown) {
    return Commands.runOnce(() -> setElevatorPosition(targetPosition, isHandDown) , this);
  }

  public void runMotor(double speed){
    m_motorElevatorLeft.set(speed);
  }

  public boolean isElevatorStalled(){
    return m_motorElevatorLeft.getOutputCurrent() >= ElevatorConstants.kStallCurrent;
  }

  public void stopElevatorMotor(){
    m_motorElevatorLeft.set(0);
  }

  public Command cmdStopElevator() {
    return Commands.runOnce(() -> stopElevatorMotor() , this);
  }

public boolean isElevatorUp() {
  return getPositionInches() > ElevatorConstants.kLevel2Inches;
}

public boolean isElevatorInPickup() {
  return getPositionInches() <= ElevatorConstants.kCoralPickup;
}

public boolean isElevatorAtCrossbar() {
  return getPositionInches() >= ElevatorConstants.kCrossbar;
}

private double getPositionInches() {
  return (( m_encoderElevatorLeft.getPosition() / ElevatorConstants.kGearRatio) * ElevatorConstants.kWinchCircumferenceInches) + ElevatorConstants.kHandStartUpInches;
  // ((m_elevatorTargetPostion - ElevatorConstants.kHandStartUpInches) / ElevatorConstants.kWinchCircumferenceInches) * ElevatorConstants.kGearRatio;
}

private void elevatorZero(){
  stopElevatorMotor();
  m_encoderElevatorLeft.setPosition(0);
}
public Command cmdElevatorZero(BooleanSupplier isHandDown){
  //return Commands.runOnce(() -> stopElevatorMotor(), this)
   // .andThen(Commands.run(() -> runMotor(ElevatorConstants.kElevatorDown) , this))
   return Commands.run(() -> setElevatorPosition(ElevatorConstants.kFloorLevel, isHandDown))
    .until(() -> m_motorElevatorLeft.getOutputCurrent() >= ElevatorConstants.kBottomCurrent)
    .andThen(() -> elevatorZero() , this)
    ;
}
}
