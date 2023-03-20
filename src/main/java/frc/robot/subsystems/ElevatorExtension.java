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

public class ElevatorExtension extends SubsystemBase {
  /** Creates a new ElevatorExtend. */
  private TalonFX leftExtensionMotor;
  private TalonFX rightExtensionMotor;

  

  public ElevatorExtension(){
    leftExtensionMotor = new TalonFX(Constants.ElevatorExtensionLeftID);        // TODO: add proper IDs for the motor
    rightExtensionMotor = new TalonFX(Constants.ElevatorExtensionRightID);      // TODO: add proper IDs for the motor
    //leftExtensionMotor.follow(rightExtensionMotor); //sets right motor to master and tells left motor to follow
    rightExtensionMotor.configFactoryDefault();
    rightExtensionMotor.setNeutralMode(NeutralMode.Brake);
    rightExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    //rightExtensionMotor.setSensorPhase(true); use if forwards and backwards is wacky
    leftExtensionMotor.setInverted(true); // if motors go wrong way set to true
    rightExtensionMotor.configForwardSoftLimitThreshold(1000); //set to whatever limits are needed
    rightExtensionMotor.configReverseSoftLimitThreshold(1000); //^
    rightExtensionMotor.configForwardSoftLimitEnable(true, 0); 
    rightExtensionMotor.configReverseSoftLimitEnable(true, 0); 
  }
  
  public void extend(double val){
    rightExtensionMotor.set(ControlMode.PercentOutput, val);
    leftExtensionMotor.set(ControlMode.PercentOutput, val);
     // TODO Adjust output to something that works
  }

  public void goToExtensionStow(){
    rightExtensionMotor.set(ControlMode.Position, Constants.Position.STANDBY.getExtend()); // might not work TODO: CHANGE VALUE
  }

  public void goToExtensionMidGoal(){
    rightExtensionMotor.set(ControlMode.Position, Constants.Position.MID.getExtend());  // might not work TODO: CHANGE VALUE
  }

  public void goToExtensionHighGoal(){
    rightExtensionMotor.set(ControlMode.Position,Constants.Position.CONEHIGH.getExtend()); // might not work TODO: CHANGE VALUE
  }

  public void stopExtension(){
    rightExtensionMotor.set(ControlMode.PercentOutput, 0);
  }
  public void resetExtension(){
  rightExtensionMotor.set(ControlMode.Position,0); // TODO: set to all the way closed
  }

  public void resetExtensionEncoder(){
    rightExtensionMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extension Motor Position: ", getExtensionMotorPosition());
  }

  public double getExtensionMotorPosition() {
      return rightExtensionMotor.getSelectedSensorPosition();
  }
}
