// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorExtension extends SubsystemBase {
  /** Creates a new ElevatorExtend. */
  private TalonFX leftExtensionMotor;
  private TalonFX rightExtensionMotor;
  private int leftLowerBound = 0; 
  private int leftUpperBound = 0;
  private int rightLowerBound = 0;
  private int rightUpperBound = 0;

  public ElevatorExtension(){
    leftExtensionMotor = new TalonFX(Constants.ElevatorExtensionLeftID);        // TODO: add proper IDs for the motor
    rightExtensionMotor = new TalonFX(Constants.ElevatorExtensionRightID);      // TODO: add proper IDs for the motor
    //leftExtensionMotor.follow(rightExtensionMotor); //sets right motor to master and tells left motor to follow
    rightExtensionMotor.configFactoryDefault();
    leftExtensionMotor.configFactoryDefault();

    rightExtensionMotor.setNeutralMode(NeutralMode.Brake);
    leftExtensionMotor.setNeutralMode(NeutralMode.Brake);

    rightExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    rightExtensionMotor.setSelectedSensorPosition(0);
    leftExtensionMotor.setSelectedSensorPosition(0);

    //rightExtensionMotor.setSensorPhase(true); use if forwards and backwards is wacky
    leftExtensionMotor.setInverted(true); // if motors go wrong way set to true

    rightExtensionMotor.configForwardSoftLimitThreshold(rightLowerBound); //set to whatever limits are needed
    rightExtensionMotor.configReverseSoftLimitThreshold(rightUpperBound); //^
    leftExtensionMotor.configForwardSoftLimitThreshold(leftLowerBound); //set to whatever limits are needed
    leftExtensionMotor.configReverseSoftLimitThreshold(leftUpperBound);

    rightExtensionMotor.configForwardSoftLimitEnable(false, 0); 
    rightExtensionMotor.configReverseSoftLimitEnable(false, 0); 
    leftExtensionMotor.configForwardSoftLimitEnable(false, 0); 
    leftExtensionMotor.configReverseSoftLimitEnable(false, 0); 
  }
  
  public void extend(double val){
    rightExtensionMotor.set(ControlMode.PercentOutput, val);
    leftExtensionMotor.set(ControlMode.PercentOutput, val);
     // TODO Adjust output to something that works
  }

  public void goToExtensionStow(){
    leftExtensionMotor.set(ControlMode.Follower, Constants.ElevatorExtensionRightID);
    rightExtensionMotor.set(ControlMode.Position, Constants.Position.STANDBY.getExtend());
  }

  public void goToExtensionMidGoal(){
    leftExtensionMotor.set(ControlMode.Follower, Constants.ElevatorExtensionRightID);
    rightExtensionMotor.set(ControlMode.Position, Constants.Position.MID.getExtend());  
  }

  public void goToExtensionHighGoal(){
    leftExtensionMotor.set(ControlMode.Follower, Constants.ElevatorExtensionRightID);
    rightExtensionMotor.set(ControlMode.Position, Constants.Position.CONEHIGH.getExtend()); 
  }

  public void stopExtension(){
    rightExtensionMotor.set(ControlMode.PercentOutput, 0);
    leftExtensionMotor.set(ControlMode.PercentOutput, 0);
  }
  public void resetExtension(){
    rightExtensionMotor.set(ControlMode.Position,0);
    leftExtensionMotor.set(ControlMode.Position,0); // TODO: set to all the way closed
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Right Extension Motor Position: ", rightExtensionMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Extension Motor Position: ", leftExtensionMotor.getSelectedSensorPosition());
  }
}
