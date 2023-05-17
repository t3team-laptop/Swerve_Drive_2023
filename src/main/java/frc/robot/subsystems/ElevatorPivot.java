// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
  public TalonFX leftPivotMotor;
  public TalonFX rightPivotMotor;

  private int leftLowerBound = 641943; 
  private int leftUpperBound = -95391;
  private int rightLowerBound = 0;
  private int rightUpperBound = 0;

  public Position rotateState;
  private Debouncer setpointDebouncer;
  private boolean rotateEnableCurrentLimit = true;
  private int rotateContinuousCurrentLimit = 35;
  private double rotatePeakCurrentLimit = 15;
  private double rotatePeakCurrentDuration = 0.1;
  private TalonFXConfiguration FXConfig = new TalonFXConfiguration();
  private SupplyCurrentLimitConfiguration rotateSupplyLimit = new SupplyCurrentLimitConfiguration(rotateEnableCurrentLimit, rotateContinuousCurrentLimit, rotatePeakCurrentLimit, rotatePeakCurrentDuration);
  private double setpointTolerance = 3000;
  public boolean isPosition = false;
  public double percentOutput = 0.0;
  public ElevatorPivot() {
    leftPivotMotor = new TalonFX(Constants.ElevatorPivotLeftID);
    rightPivotMotor = new TalonFX(Constants.ElevatorPivotRightID);
    FXConfig.slot0.kP = Constants.Elevator.elevatorPivotKP;
    FXConfig.slot0.kI = Constants.Elevator.elevatorPivotKI;
    FXConfig.slot0.kD = Constants.Elevator.elevatorPivotKD;
    FXConfig.slot0.kF = Constants.Elevator.elevatorPivotKF;
    FXConfig.supplyCurrLimit = rotateSupplyLimit;
    FXConfig.openloopRamp = 0.25;
    FXConfig.closedloopRamp = 0.2;
    configMotors();




    setpointDebouncer = new Debouncer(0.2, DebounceType.kRising);
    rotateState = Position.OFF;

    rightPivotMotor.configForwardSoftLimitThreshold(rightLowerBound); //set to whatever limits are needed
    rightPivotMotor.configReverseSoftLimitThreshold(rightUpperBound); //^
    leftPivotMotor.configForwardSoftLimitThreshold(leftLowerBound); //set to whatever limits are needed
    leftPivotMotor.configReverseSoftLimitThreshold(leftUpperBound);

    rightPivotMotor.configForwardSoftLimitEnable(false, 0); 
    rightPivotMotor.configReverseSoftLimitEnable(false, 0); 
    leftPivotMotor.configForwardSoftLimitEnable(false, 0); 
    leftPivotMotor.configReverseSoftLimitEnable(false, 0); 
  }

  public void configMotors(){
    leftPivotMotor.configFactoryDefault();
    leftPivotMotor.configAllSettings(FXConfig);
    leftPivotMotor.setInverted(true);
    leftPivotMotor.setNeutralMode(NeutralMode.Brake);
    leftPivotMotor.setSelectedSensorPosition(0);

    rightPivotMotor.configFactoryDefault();
    rightPivotMotor.configAllSettings(FXConfig);
    rightPivotMotor.setInverted(false);
    rightPivotMotor.setNeutralMode(NeutralMode.Brake);
    rightPivotMotor.setSelectedSensorPosition(0);
    rightPivotMotor.follow(leftPivotMotor);
    rightPivotMotor.set(ControlMode.Follower, Constants.ElevatorPivotLeftID);
  }

  @Override
  public void periodic(){
    if(rotateState.equals(Position.OFF)) leftPivotMotor.set(ControlMode.PercentOutput, 0);
    SmartDashboard.putNumber("Left Pivot Motor Position: ", leftPivotMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Pivot Motor Position: ", rightPivotMotor.getSelectedSensorPosition());
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

  public void setPivotPosition(double position, boolean bol){
    if(bol){
      leftPivotMotor.set(ControlMode.Position, position);}
    else{
      leftPivotMotor.set(ControlMode.PercentOutput, position);
    }
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
