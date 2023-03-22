// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;

public class ElevatorPivot extends SubsystemBase {
  /** Creates a new ElevatorPivot. */
  private TalonFX leftPivotMotor;
  private TalonFX rightPivotMotor;

  private int leftLowerBound = 0; 
  private int leftUpperBound = 0;
  private int rightLowerBound = 0;
  private int rightUpperBound = 0;

  private Position rotateState;
  private Debouncer setpointDebouncer;
  private boolean rotateEnableCurrentLimit = true;
  private int rotateContinuousCurrentLimit = 35;
  private double rotatePeakCurrentLimit = 45;
  private double rotatePeakCurrentDuration = 0.1;
  private TalonFXConfiguration FXConfig = new TalonFXConfiguration();
  private SupplyCurrentLimitConfiguration rotateSupplyLimit = new SupplyCurrentLimitConfiguration(rotateEnableCurrentLimit, rotateContinuousCurrentLimit, rotatePeakCurrentLimit, rotatePeakCurrentDuration);
  private double setpointTolerance = 3000;
  public ElevatorPivot() {
    leftPivotMotor = new TalonFX(Constants.ElevatorPivotLeftID);
    rightPivotMotor = new TalonFX(Constants.ElevatorPivotRightID);
    FXConfig.slot0.kP = Constants.Elevator.elevatorKP;
    FXConfig.slot0.kI = Constants.Elevator.elevatorKI;
    FXConfig.slot0.kD = Constants.Elevator.elevatorKD;
    FXConfig.slot0.kF = Constants.Elevator.elevatorKF;
    FXConfig.supplyCurrLimit = rotateSupplyLimit;
    FXConfig.openloopRamp = 0.25;
    FXConfig.closedloopRamp = 0.2;
    configMotors();




    setpointDebouncer = new Debouncer(0.2, DebounceType.kRising);
    rotateState = Position.STANDBY;
  }

  public void configMotors(){
    leftPivotMotor.configFactoryDefault();
    leftPivotMotor.configAllSettings(FXConfig);
    leftPivotMotor.setInverted(false);
    leftPivotMotor.setNeutralMode(NeutralMode.Brake);
    leftPivotMotor.setSelectedSensorPosition(0);

    rightPivotMotor.configFactoryDefault();
    rightPivotMotor.configAllSettings(FXConfig);
    rightPivotMotor.setInverted(true);
    rightPivotMotor.setNeutralMode(NeutralMode.Brake);
    rightPivotMotor.setSelectedSensorPosition(0);
    rightPivotMotor.follow(leftPivotMotor);
    rightPivotMotor.set(ControlMode.Follower, Constants.ElevatorPivotLeftID);
  }

  @Override
  public void periodic(){
    if(rotateState.equals(Position.OFF)) leftPivotMotor.set(ControlMode.PercentOutput, 0);
    SmartDashboard.putNumber("Pivot Motor Position: ", leftPivotMotor.getSelectedSensorPosition());
    setPivotPosition(rotateState.getPivot());
  }

  public double getEncoderPosition(){
    return leftPivotMotor.getSelectedSensorPosition();
  }

  public void setArmState(Position pivotState){
    this.rotateState = pivotState;
  }

  public void resetIntegrator(){
    System.out.println(leftPivotMotor.getIntegralAccumulator());
    leftPivotMotor.setIntegralAccumulator(0);
  }

  public void setPivotPosition(double position){
    leftPivotMotor.set(ControlMode.Position, position);
  }

  public boolean atDesiredSetPoint(){
    if(rotateState.equals(Position.OFF)) return true;
    return isAtSetpoint(rotateState.getPivot(), setpointTolerance);
  }

  public boolean atDebouncedSetPoint(){
    return setpointDebouncer.calculate(atDesiredSetPoint());
  }

  public boolean isAtSetpoint(double setpoint){
    return getEncoderPosition() == setpoint;
  }

  public boolean isAtSetpoint(double setpoint, double tolerance){
    return Math.abs(setpoint-getEncoderPosition()) < tolerance;
  }
}
