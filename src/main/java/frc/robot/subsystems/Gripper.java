// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  DoubleSolenoid mSolenoid;
  Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  /** Creates a new Gripper. */
  public Gripper() {
    mSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    compressor.enableDigital();
  }

  public void disable(){
    mSolenoid.set(Value.kOff);
    compressor.disable();
  }
  public void openGripper(){
    mSolenoid.set(Value.kForward);
  }
  public void closeGripper(){
    mSolenoid.set(Value.kReverse);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
