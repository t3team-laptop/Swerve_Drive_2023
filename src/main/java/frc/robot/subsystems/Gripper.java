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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  private PneumaticHub pneumaticHub;
  private Compressor compressor;
  private DoubleSolenoid mSolenoid;
  /** Creates a new Gripper. */
  public Gripper() {
    pneumaticHub = new PneumaticHub();
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    mSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    compressor.enableDigital();
    open();
  }

  public void close(){
    mSolenoid.set(Value.kForward);
  }
  public void open(){
    mSolenoid.set(Value.kReverse);
  }
  @Override
  public void periodic() {
  }
}
