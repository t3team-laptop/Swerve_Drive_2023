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

    rightPivotMotor.configForwardSoftLimitThreshold(rightLowerBound); //set to whatever limits are needed
    rightPivotMotor.configReverseSoftLimitThreshold(rightUpperBound); //^
    leftPivotMotor.configForwardSoftLimitThreshold(leftLowerBound); //set to whatever limits are needed
    leftPivotMotor.configReverseSoftLimitThreshold(leftUpperBound);

    rightPivotMotor.configForwardSoftLimitEnable(true, 0); 
    rightPivotMotor.configReverseSoftLimitEnable(true, 0); 
    leftPivotMotor.configForwardSoftLimitEnable(true, 0); 
    leftPivotMotor.configReverseSoftLimitEnable(true, 0); 
  
  }
  public void manualPivot(double val){
    rightPivotMotor.set(ControlMode.PercentOutput, val); 
    leftPivotMotor.set(ControlMode.PercentOutput, val); // TODO Adjust output to something that works
  }

  public void goToPivotHighGoal(){
    leftPivotMotor.set(ControlMode.Follower, Constants.ElevatorExtensionRightID);
    rightPivotMotor.set(ControlMode.Position,Constants.Position.CONEHIGH.getPivot());
  }

  public void goToPivotMidGoal(){
    leftPivotMotor.set(ControlMode.Follower, Constants.ElevatorExtensionRightID);
    rightPivotMotor.set(ControlMode.Position,Constants.Position.CONEMID.getPivot());//TODO: CONFIG POSITION
  }

  public void goToPivotStow(){
    leftPivotMotor.set(ControlMode.Follower, Constants.ElevatorExtensionRightID);
    rightPivotMotor.set(ControlMode.Position,Constants.Position.STANDBY.getPivot());//TODO: CONFIG POSITION
  }
  public void stopPivot(){
    rightPivotMotor.set(ControlMode.PercentOutput,0);
    leftPivotMotor.set(ControlMode.PercentOutput,0);
  }
  public void goToPivotGround(){
    leftPivotMotor.set(ControlMode.Follower, Constants.ElevatorExtensionRightID);
    rightPivotMotor.set(ControlMode.Position, Constants.Position.FLOOR.getPivot()); //TODO: CONFIG POSITION
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Right Pivot Motor Position: ", rightPivotMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Pivot Motor Position: ", leftPivotMotor.getSelectedSensorPosition());
  }
}
