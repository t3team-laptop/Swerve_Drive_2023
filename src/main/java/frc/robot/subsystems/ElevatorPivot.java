// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Position;
import frc.robot.Constants;

public class ElevatorPivot extends SubsystemBase {
  /** Creates a new ElevatorPivot. */
  private TalonFX leftPivotMotor;
  private TalonFX rightPivotMotor;
  private int leftLowerBound = 0; 
  private int leftUpperBound = 0;
  private int rightLowerBound = 0;
  private int rightUpperBound = 0;
  
  public static double currentPosition;

  private PIDController pidController;

  public ElevatorPivot() {
    leftPivotMotor = new TalonFX(Constants.ElevatorPivotLeftID);
    rightPivotMotor = new TalonFX(Constants.ElevatorPivotRightID);
    //leftPivotMotor.follow(rightPivotMotor);
    rightPivotMotor.configFactoryDefault();
    leftPivotMotor.configFactoryDefault();

    rightPivotMotor.setNeutralMode(NeutralMode.Brake);
    leftPivotMotor.setNeutralMode(NeutralMode.Brake);

    rightPivotMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftPivotMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    rightPivotMotor.setSelectedSensorPosition(0);
    leftPivotMotor.setSelectedSensorPosition(0);

    //rightPivotMotor.setSensorPhase(true); use if motor forwards and backwards is wack
    rightPivotMotor.setInverted(true); 
    rightPivotMotor.follow(leftPivotMotor);
    
    rightPivotMotor.configForwardSoftLimitThreshold(rightLowerBound); //set to whatever limits are needed
    rightPivotMotor.configReverseSoftLimitThreshold(rightUpperBound); //^
    leftPivotMotor.configForwardSoftLimitThreshold(leftLowerBound); //set to whatever limits are needed
    leftPivotMotor.configReverseSoftLimitThreshold(leftUpperBound);

    rightPivotMotor.configForwardSoftLimitEnable(false, 0); 
    rightPivotMotor.configReverseSoftLimitEnable(false, 0); 
    leftPivotMotor.configForwardSoftLimitEnable(false, 0); 
    leftPivotMotor.configReverseSoftLimitEnable(false, 0); 
    
    pidController = new PIDController(Constants.Elevator.elevatorKP, Constants.Elevator.elevatorKI, Constants.Elevator.elevatorKD);
    pidController.setSetpoint(0);
    pidController.setTolerance(.25);

    setPosition(Position.STANDBY.getPivot());
  }
     
  public void manualPivot(double val){
    //rightPivotMotor.set(ControlMode.PercentOutput, val); 
    leftPivotMotor.set(ControlMode.PercentOutput, val); // TODO Adjust output to something that works
  }
 
  public void setPosition(double position){
    currentPosition = position;
  }

  public double getPosition(){
    return currentPosition;
  }

  public void move(double val){
    leftPivotMotor.set(ControlMode.PercentOutput, val);
  }

  public boolean reachedSetpoint(double distance){
    return pidController.getPositionTolerance() >= Math.abs(currentPosition - distance);
  }
 
  
  public void stopPivot(){
    rightPivotMotor.set(ControlMode.PercentOutput,0);
    leftPivotMotor.set(ControlMode.PercentOutput,0);
  }
  
  public double getEncoderPosition(){
    return (leftPivotMotor.getSelectedSensorPosition() + rightPivotMotor.getSelectedSensorPosition()) /2.0;
  }
  
  public boolean atSetpoint(){
    return pidController.atSetpoint();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Elevator at setpoint", atSetpoint());
    SmartDashboard.putNumber("Elevator Position", getEncoderPosition());
    SmartDashboard.putNumber("Elevator Goal Position", currentPosition);

        move(pidController.calculate(getEncoderPosition(), currentPosition));
  }
}
