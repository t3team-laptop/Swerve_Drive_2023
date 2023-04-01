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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;

public class ElevatorExtension extends SubsystemBase {
  /** Creates a new ElevatorExtend. */
  public TalonFX leftExtensionMotor;
  private TalonFX rightExtensionMotor;
  private int leftLowerBound = 1; 
  private int leftUpperBound = -117899; 
  private int rightLowerBound = 0;
  private int rightUpperBound = 0;

  private Position rotateState;
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
  public ElevatorExtension(){
    leftExtensionMotor = new TalonFX(Constants.ElevatorExtensionLeftID);       
    rightExtensionMotor = new TalonFX(Constants.ElevatorExtensionRightID);  
   // leftExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //rightExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    FXConfig.slot0.kP = Constants.Elevator.elevatorExtensionKP;
    FXConfig.slot0.kI = Constants.Elevator.elevatorExtensionKI;
    FXConfig.slot0.kD = Constants.Elevator.elevatorExtensionKD;
    FXConfig.slot0.kF = Constants.Elevator.elevatorPivotKF;
    FXConfig.supplyCurrLimit = rotateSupplyLimit;
    FXConfig.openloopRamp = 0.25;
    FXConfig.closedloopRamp = 0.2;
    configMotors();




    setpointDebouncer = new Debouncer(0.2, DebounceType.kRising);
    rotateState = Position.OFF;

    rightExtensionMotor.configForwardSoftLimitThreshold(rightLowerBound); //set to whatever limits are needed
    rightExtensionMotor.configReverseSoftLimitThreshold(rightUpperBound); //^
    leftExtensionMotor.configForwardSoftLimitThreshold(leftLowerBound); //set to whatever limits are needed
    leftExtensionMotor.configReverseSoftLimitThreshold(leftUpperBound);

    rightExtensionMotor.configForwardSoftLimitEnable(false, 0); 
    rightExtensionMotor.configReverseSoftLimitEnable(false, 0); 
    leftExtensionMotor.configForwardSoftLimitEnable(false, 0); 
    leftExtensionMotor.configReverseSoftLimitEnable(false, 0); 
  }

  public void configMotors(){
    leftExtensionMotor.configFactoryDefault();
    leftExtensionMotor.configAllSettings(FXConfig);
    leftExtensionMotor.setInverted(true);
    leftExtensionMotor.setNeutralMode(NeutralMode.Brake);
    leftExtensionMotor.setSelectedSensorPosition(0);

    rightExtensionMotor.configFactoryDefault();
    rightExtensionMotor.configAllSettings(FXConfig);
    rightExtensionMotor.setInverted(false);
    rightExtensionMotor.setNeutralMode(NeutralMode.Brake);
    rightExtensionMotor.setSelectedSensorPosition(0);
    rightExtensionMotor.follow(leftExtensionMotor);
    rightExtensionMotor.set(ControlMode.Follower, Constants.ElevatorExtensionLeftID);
  }
  
  @Override
  public void periodic(){
    if(rotateState.equals(Position.OFF)) leftExtensionMotor.set(ControlMode.PercentOutput, 0);
    //else if(isPosition){setExtensionPosition(rotateState.getPivot(), true);}
    //else{setExtensionPosition(percentOutput, false);}
    SmartDashboard.putNumber("Left Extension Motor Position: ", leftExtensionMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Extension Motor Position: ", rightExtensionMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("setpoint for Extension Motor: ", rotateState.getExtend());
    //setExtensionPosition(rotateState.getExtend(), true);
  }

  public double getEncoderPosition(){
    return leftExtensionMotor.getSelectedSensorPosition();
  }

  public void setArmState(Position state){
    this.rotateState = state;
  }

  public void resetIntegrator(){
    System.out.println(leftExtensionMotor.getIntegralAccumulator());
    leftExtensionMotor.setIntegralAccumulator(0);
  }
  
  public void setExtensionPosition(double position, boolean bol){
    if(bol){
    leftExtensionMotor.set(ControlMode.Position, position);}
    else{
      leftExtensionMotor.set(ControlMode.PercentOutput, position);
    }
  }

  public boolean atDesiredSetPoint(){
    if(rotateState.equals(Position.OFF)) return true;
    return isAtSetpoint(rotateState.getExtend(), setpointTolerance);
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

