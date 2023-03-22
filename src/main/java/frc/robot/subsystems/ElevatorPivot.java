// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorPivot extends SubsystemBase {
  /** Creates a new ElevatorPivot. */
  private TalonFX leftPivotMotor;
  private TalonFX rightPivotMotor;

  private int leftLowerBound = 0; 
  private int leftUpperBound = 0;
  private int rightLowerBound = 0;
  private int rightUpperBound = 0;

  private double motorPGain = .01;
  private double motorIGain = 0;
  private double motorDGain = 0;
  private int kConfigurationTimeoutLimit = 5000;
  private int kPivotMotorPIDSlot = 0;


  public ElevatorPivot() {
    leftPivotMotor = new TalonFX(Constants.ElevatorPivotLeftID);
    rightPivotMotor = new TalonFX(Constants.ElevatorPivotRightID);
    //leftPivotMotor.follow(rightPivotMotor);
    rightPivotMotor.configFactoryDefault();
    rightPivotMotor.setNeutralMode(NeutralMode.Brake);
    rightPivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

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

    rightPivotMotor.config_kP(kPivotMotorPIDSlot, motorPGain, kConfigurationTimeoutLimit);
    rightPivotMotor.config_kI(kPivotMotorPIDSlot, motorIGain, kConfigurationTimeoutLimit);
    rightPivotMotor.config_kD(kPivotMotorPIDSlot, motorIGain, kConfigurationTimeoutLimit);
    leftPivotMotor.config_kP(kPivotMotorPIDSlot, motorPGain, kConfigurationTimeoutLimit);
    leftPivotMotor.config_kI(kPivotMotorPIDSlot, motorIGain, kConfigurationTimeoutLimit);
    leftPivotMotor.config_kD(kPivotMotorPIDSlot, motorIGain, kConfigurationTimeoutLimit);


  }
  public void pivotUp(double val){
    //rightPivotMotor.set(ControlMode.Position, val); 
    leftPivotMotor.set(ControlMode.Position, leftPivotMotor.getSelectedSensorPosition() + val); // TODO Adjust output to something that works
  }

  public void goToPivotHighGoal(){
    leftPivotMotor.set(ControlMode.Position,Constants.Position.CONEHIGH.getPivot()); //TODO: CONFIG POSITION
  }

  public void goToPivotMidGoal(){
    leftPivotMotor.set(ControlMode.Position,Constants.Position.CONEMID.getPivot());//TODO: CONFIG POSITION
  }

  public void goToPivotStow(){
    leftPivotMotor.set(ControlMode.Position,Constants.Position.STANDBY.getPivot());//TODO: CONFIG POSITION
  }
  public void stopPivot(){
    leftPivotMotor.set(ControlMode.PercentOutput,0);
  }
  public void resetPivotEncoder(){
    leftPivotMotor.setSelectedSensorPosition(0);
  }
  public void goToPivotGround(){
    leftPivotMotor.set(ControlMode.Position, Constants.Position.FLOOR.getPivot()); //TODO: CONFIG POSITION
  }
  public void resetPivot(){
    leftPivotMotor.set(ControlMode.Position, 0); //TODO: CONFIG POSITION TO ALL THE WAY UP.
  }
  public double getPivotMotorPosition() {
    return leftPivotMotor.getSelectedSensorPosition();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Motor Position: ", getPivotMotorPosition());
  }
}
