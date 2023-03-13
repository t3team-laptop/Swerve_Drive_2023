// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorPivot extends SubsystemBase {
  /** Creates a new ElevatorPivot. */
  private TalonFX leftPivotMotor;
  private TalonFX rightPivotMotor;

  public ElevatorPivot() {
    leftPivotMotor = new TalonFX(Constants.ElevatorPivotLeftID);
    rightPivotMotor = new TalonFX(Constants.ElevatorPivotRightID);
    leftPivotMotor.follow(rightPivotMotor);
    rightPivotMotor.configFactoryDefault();
    rightPivotMotor.setNeutralMode(NeutralMode.Brake);
    rightPivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    //rightPivotMotor.setSensorPhase(true); use if motor forwards and backwards is wack
    //rightPivotMotor.setInverted(true); set true if motor spins wrong way

    rightPivotMotor.configForwardSoftLimitThreshold(1000); //set to whatever limits are needed
    rightPivotMotor.configReverseSoftLimitThreshold(1000); //^
    rightPivotMotor.configForwardSoftLimitEnable(true, 0); 
    rightPivotMotor.configReverseSoftLimitEnable(true, 0); 
  
  }
  public void manual(double pivotVelocity){
    rightPivotMotor.set(ControlMode.Velocity,pivotVelocity);
  }

  public void goToPivotScore(){
    rightPivotMotor.set(ControlMode.Position,0); //TODO: CONFIG POSITION
  }

  public void goToPivotPreScore(){
    rightPivotMotor.set(ControlMode.Position,0);//TODO: CONFIG POSITION
  }

  public void goToPivotStow(){
    rightPivotMotor.set(ControlMode.Position,0);//TODO: CONFIG POSITION
  }
  public void stopPivot(){
    rightPivotMotor.set(ControlMode.PercentOutput,0);
  }
  public void resetPivotEncoder(){
    rightPivotMotor.setSelectedSensorPosition(0);
  }
  public void goToPivotGround(){
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
