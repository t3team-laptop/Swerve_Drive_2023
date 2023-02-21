// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

public class ElevatorPivot extends SubsystemBase {
  private static final double kPivotPowerOut = 1.0;
  private static final double kPivotPowerIn = -0.7;
  private static final double kPivotBoostAmount = -3;
  private static final double kPivotBoost2Amount = -15;
  private static final int kPIDLoopIdx = 0; // i dont know what value this should be it said default to 
  private static final int kMaxOutput = 1; // i also have no fucking clue what the value should be
  private static final double kI =0;
  private static final double kD =0;
  private static final double kF =0;
  private static final double kP = 0;
  private static final double kPivotCLRampRate = 0.5;

  private static ElevatorPivot mInstance;

  public static ElevatorPivot getInstance() {
    if (mInstance == null) {
      mInstance = new ElevatorPivot();
    }
    return mInstance;
  }

  private TalonFX mPivotMotor;
  //private SparkMaxPIDController mPivotPIDController;  i have no clue how to do the pid stuff

  private PeriodicIO mPeriodicIO = new PeriodicIO();

  /** Creates a new ElevatorPivot. */
  public ElevatorPivot() {
    mPivotMotor = new TalonFX(Constants.kElevatorPivotMotorId);
    mPivotMotor.configFactoryDefault();
    mPivotMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    mPivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,kPIDLoopIdx,Constants.kTimeoutMs);
    //mPivotPIDController = mPivotMotor.getPIDController();

    mPivotMotor.configNominalOutputForward(0,Constants.kTimeoutMs);
    mPivotMotor.configNominalOutputReverse(0,Constants.kTimeoutMs);
    mPivotMotor.configPeakOutputForward(kMaxOutput,Constants.kTimeoutMs);
    mPivotMotor.configPeakOutputReverse(kMaxOutput,Constants.kTimeoutMs);


    mPivotMotor.config_kP(kPIDLoopIdx,kI,Constants.kTimeoutMs);
    mPivotMotor.config_kI(kPIDLoopIdx,kD,Constants.kTimeoutMs);
    mPivotMotor.config_kD(kPIDLoopIdx,kF,Constants.kTimeoutMs);
    mPivotMotor.config_kF(kPIDLoopIdx,kP,Constants.kTimeoutMs);


    //mPivotMotor.setClosedLoopRampRate(kPivotCLRampRate);

    mPivotMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen);

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
        //mPivotPIDController.setReference(mPeriodicIO.pivot_target + kPivotBoostAmount,
            //CANSparkMax.ControlType.kPosition);
      } else if (mPeriodicIO.is_pivot_boosted2) {
        //mPivotPIDController.setReference(mPeriodicIO.pivot_target + kPivotBoost2Amount,
            //CANSparkMax.ControlType.kPosition);
      } else {
        //mPivotPIDController.setReference(mPeriodicIO.pivot_target,
           // CANSparkMax.ControlType.kPosition);
      }
    } else {
      mPivotMotor.set(ControlMode.PercentOutput,mPeriodicIO.pivot_power);
    }
  }

  public void stop() {
    stopPivot();
  }

  public void stopPivot() {
    if (!mPeriodicIO.is_pivot_pos_control) {
      mPeriodicIO.pivot_power = 0.0;
      mPivotMotor.set(ControlMode.PercentOutput,0.0);
    }
  }

  public void resetPivotEncoder() {
    mPivotMotor.setSelectedSensorPosition(0);
  }

 
}