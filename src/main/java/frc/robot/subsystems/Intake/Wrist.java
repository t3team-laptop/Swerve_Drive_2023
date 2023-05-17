// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants.Position;

public class Wrist extends SubsystemBase {
  public double position;
  public TalonFX wristMotor;
  
  private int lowerBound = 0;
  private int upperBound = -227912;

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
  /** Creates a new Wrist. */
  public Wrist() {
    wristMotor = new TalonFX(25);
    FXConfig.slot0.kP = .05;
    FXConfig.slot0.kI = 0;
    FXConfig.slot0.kD = 0.0;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(.05, 0, 0.0);
    FXConfig.supplyCurrLimit = rotateSupplyLimit;
    FXConfig.openloopRamp = 0.25;
    FXConfig.closedloopRamp = .2;
    configMotors();

    setpointDebouncer = new Debouncer(0.2, DebounceType.kRising);
    rotateState = Position.OFF;

    wristMotor.configForwardSoftLimitThreshold(lowerBound); //set to whatever limits are needed
    wristMotor.configReverseSoftLimitThreshold(upperBound); //^


    wristMotor.configForwardSoftLimitEnable(false, 0); 
    wristMotor.configReverseSoftLimitEnable(false, 0); 
  }

  public void configMotors(){
    wristMotor.configFactoryDefault();
    wristMotor.configAllSettings(FXConfig);
    wristMotor.setInverted(false);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.setSelectedSensorPosition(0);
  }
  @Override
  public void periodic() {
    if(rotateState.equals(Position.OFF)) wristMotor.set(ControlMode.PercentOutput, 0);
    SmartDashboard.putNumber("Wrist Motor Position: ", wristMotor.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }
  
  public double getEncoderPosition(){
    return wristMotor.getSelectedSensorPosition();
  }

  public void setArmState(Position pivotState){
    this.rotateState = pivotState;
  }

  public void resetIntegrator(){
    System.out.println(wristMotor.getIntegralAccumulator());
    wristMotor.setIntegralAccumulator(0);
  }

  public void setPivotPosition(double pos, boolean bol){
    position = pos;
    if(bol){
      wristMotor.set(ControlMode.Position, pos);}
    else{
      wristMotor.set(ControlMode.PercentOutput, position);
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
