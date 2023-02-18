// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

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

  private CANSparkMax mExtensionMotor;
  public RelativeEncoder mExtensionEncoder;
  private SparkMaxPIDController mExtensionPIDController;
  private SparkMaxLimitSwitch mExtensionLowerLimit;
  private SparkMaxLimitSwitch mExtensionUpperLimit;

  private PeriodicIO mPeriodicIO = new PeriodicIO();

  /** Creates a new ElevatorExtension. */
  public ElevatorExtension() {
    mExtensionMotor = new CANSparkMax(Constants.kElevatorExtensionMotorId, MotorType.kBrushless);
    mExtensionMotor.restoreFactoryDefaults();
    mExtensionMotor.setIdleMode(IdleMode.kBrake);
    mExtensionMotor.setInverted(true);
    mExtensionPIDController = mExtensionMotor.getPIDController();
    mExtensionEncoder = mExtensionMotor.getEncoder();

    mExtensionPIDController.setP(0.1);
    mExtensionPIDController.setI(1e-8);
    mExtensionPIDController.setD(1);
    mExtensionPIDController.setIZone(0);
    mExtensionPIDController.setFF(0);
    mExtensionPIDController.setOutputRange(kExtensionPowerIn, kExtensionPowerOut);
    mExtensionMotor.setClosedLoopRampRate(kExtensionCLRampRate);

    mExtensionLowerLimit = mExtensionMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    mExtensionUpperLimit = mExtensionMotor.getForwardLimitSwitch(Type.kNormallyOpen);

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


  @Override
  public void periodic() {
    writePeriodicOutputs();
  }

  public void writePeriodicOutputs() {
    if (mPeriodicIO.is_extension_pos_control) {
      mExtensionPIDController.setReference(mPeriodicIO.extension_target, CANSparkMax.ControlType.kPosition);
    } else {
      mExtensionMotor.set(mPeriodicIO.extension_power);
    }
  }

  public void stop() {
    stopExtension();
  }

  public void stopExtension() {
    if (!mPeriodicIO.is_extension_pos_control) {
      mPeriodicIO.extension_power = 0.0;
      mExtensionMotor.set(0.0);
    }
  }

  public void resetExtensionEncoder() {
    mExtensionEncoder.setPosition(0);
  }

  public void outputTelemetry() {
    // Extension telemetry
    SmartDashboard.putNumber("Extension motor power:", mPeriodicIO.extension_power);
    SmartDashboard.putNumber("Extension encoder count:", mExtensionEncoder.getPosition());
    SmartDashboard.putBoolean("Extension lower limit:", mExtensionLowerLimit.isPressed());
    SmartDashboard.putBoolean("Extension Upper limit:", mExtensionUpperLimit.isPressed());
  }
}
