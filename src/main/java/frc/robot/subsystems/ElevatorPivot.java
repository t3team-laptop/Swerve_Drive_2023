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

public class ElevatorPivot extends SubsystemBase {
  private static final double kPivotPowerOut = 1.0;
  private static final double kPivotPowerIn = -0.7;
  private static final double kPivotBoostAmount = -3;
  private static final double kPivotBoost2Amount = -15;

  private static final double kPivotCLRampRate = 0.5;

  private static ElevatorPivot mInstance;

  public static ElevatorPivot getInstance() {
    if (mInstance == null) {
      mInstance = new ElevatorPivot();
    }
    return mInstance;
  }

  private CANSparkMax mPivotMotor;
  private RelativeEncoder mPivotEncoder;
  private SparkMaxPIDController mPivotPIDController;
  private SparkMaxLimitSwitch mPivotLowerLimit;

  private PeriodicIO mPeriodicIO = new PeriodicIO();

  /** Creates a new ElevatorPivot. */
  public ElevatorPivot() {
    mPivotMotor = new CANSparkMax(Constants.kElevatorPivotMotorId, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotPIDController = mPivotMotor.getPIDController();
    mPivotEncoder = mPivotMotor.getEncoder();

    mPivotPIDController.setP(0.1);
    mPivotPIDController.setI(1e-8);
    mPivotPIDController.setD(1);
    mPivotPIDController.setIZone(0);
    mPivotPIDController.setFF(0);
    mPivotPIDController.setOutputRange(kPivotPowerIn, kPivotPowerOut);
    mPivotMotor.setClosedLoopRampRate(kPivotCLRampRate);

    mPivotLowerLimit = mPivotMotor.getForwardLimitSwitch(Type.kNormallyOpen);

    mPeriodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    double pivot_power = 0.0;
    double pivot_target = 0.0;
    boolean is_pivot_pos_control = false;
    boolean is_pivot_boosted = false;
    boolean is_pivot_boosted2 = false;
  }

  public void manual(double pivotPower) {
    mPeriodicIO.is_pivot_pos_control = false;
    mPeriodicIO.pivot_power = pivotPower;
  }

  public void goToPivotGround() {
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotGroundCount;
  }

  public void goToPivotScore() {
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotScoreCount;
  }

  public void goToPivotPreScore() {
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotPreScoreCount;
  }

  public void goToPivotStow() {
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotStowCount;
  }
  
  public void boostPivot(boolean boost) {
    mPeriodicIO.is_pivot_boosted = boost;
  }

  public void boostPivot2(boolean boost) {
    mPeriodicIO.is_pivot_boosted2 = boost;
  }

  @Override
  public void periodic() {
    writePeriodicOutputs();
  }

  public void writePeriodicOutputs() {
    if (mPeriodicIO.is_pivot_pos_control) {
      if (mPeriodicIO.is_pivot_boosted) {
        mPivotPIDController.setReference(mPeriodicIO.pivot_target + kPivotBoostAmount,
            CANSparkMax.ControlType.kPosition);
      } else if (mPeriodicIO.is_pivot_boosted2) {
        mPivotPIDController.setReference(mPeriodicIO.pivot_target + kPivotBoost2Amount,
            CANSparkMax.ControlType.kPosition);
      } else {
        mPivotPIDController.setReference(mPeriodicIO.pivot_target,
            CANSparkMax.ControlType.kPosition);
      }
    } else {
      mPivotMotor.set(mPeriodicIO.pivot_power);
    }
  }

  public void stop() {
    stopPivot();
  }

  public void stopPivot() {
    if (!mPeriodicIO.is_pivot_pos_control) {
      mPeriodicIO.pivot_power = 0.0;
      mPivotMotor.set(0.0);
    }
  }

  public void resetPivotEncoder() {
    mPivotEncoder.setPosition(0);
  }

  public void outputTelemetry() {
    // Pivot telemetry
    SmartDashboard.putNumber("Pivot motor power:", mPeriodicIO.pivot_power);
    SmartDashboard.putNumber("Pivot encoder count:", mPivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot PID target:", mPeriodicIO.pivot_power);
    SmartDashboard.putBoolean("Pivot lower limit:", mPivotLowerLimit.isPressed());
  }
}
