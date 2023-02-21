// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorExtension extends SubsystemBase {
  private static final double kExtensionPowerOut = 0.6;
  private static final double kExtensionPowerIn = -0.6;
  private static final double kExtensionCLRampRate = 0.5;

  private static ElevatorExtension mInstance;

  public static ElevatorExtension getInstance() {
    if (mInstance == null) {
      mInstance = new ElevatorExtension();
    }
    return mInstance;
  }

  private TalonFX mExtensionMotor;

  private PeriodicIO mPeriodicIO = new PeriodicIO();

  /** Creates a new ElevatorExtension. */
  public ElevatorExtension() {
    mExtensionMotor = new TalonFX(Constants.kElevatorExtensionMotorId);
    mExtensionMotor.configFactoryDefault();
    mExtensionMotor.setNeutralMode(NeutralMode.Brake);
    mExtensionMotor.setInverted(true);
    mExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    mExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    mExtensionMotor.setSensorPhase(true);
    
    
    
    mExtensionMotor.configClosedloopRamp(kExtensionCLRampRate);

     mExtensionMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen);
     mExtensionMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen);

    mPeriodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    double extension_power = 0.0;
    double extension_target = 0.0;
    boolean is_extension_pos_control = false;
  }

  public void extend(double extensionPowerOut) {
    mPeriodicIO.is_extension_pos_control = false;
    mPeriodicIO.extension_power = extensionPowerOut;
  }

  public void retract(double extensionPowerIn) {
    mPeriodicIO.is_extension_pos_control = false;
    mPeriodicIO.extension_power = extensionPowerIn;
  }

  public void goToExtensionStow() {
    mPeriodicIO.is_extension_pos_control = true;
    mPeriodicIO.extension_target = Constants.kExtensionStowCount;
  }

  public void goToExtensionMidGoal() {
    mPeriodicIO.is_extension_pos_control = true;
    mPeriodicIO.extension_target = Constants.kExtensionMidGoalCount;
  }

  public void goToExtensionHighGoal() {
    mPeriodicIO.is_extension_pos_control = true;
    mPeriodicIO.extension_target = Constants.kExtensionHighGoalCount;
  }
  public TalonFX getExtensionMotor(){
    return mExtensionMotor;
  }


  @Override
  public void periodic() {
    writePeriodicOutputs();
  }

  public void writePeriodicOutputs() {
    if (mPeriodicIO.is_extension_pos_control) {
     // mExtensionPIDController.setReference(mPeriodicIO.extension_target, CANSparkMax.ControlType.kPosition);
    } else {
      mExtensionMotor.set(ControlMode.PercentOutput,mPeriodicIO.extension_power);
    }
  }

  public void stop() {
    stopExtension();
  }

  public void stopExtension() {
    if (!mPeriodicIO.is_extension_pos_control) {
      mPeriodicIO.extension_power = 0.0;
      mExtensionMotor.set(ControlMode.PercentOutput,0.0);
    }
  }

  public void resetExtensionEncoder() {
    mExtensionMotor.setSelectedSensorPosition(0);
  }

  public void outputTelemetry() {
    // Extension telemetry
    SmartDashboard.putNumber("Extension motor power:", mPeriodicIO.extension_power);
    SmartDashboard.putBoolean("Extension lower limit:", mExtensionMotor.getStatorCurrent() == 0);
    SmartDashboard.putBoolean("Extension Upper limit:", mExtensionMotor.getStatorCurrent() == 0);
  }
}