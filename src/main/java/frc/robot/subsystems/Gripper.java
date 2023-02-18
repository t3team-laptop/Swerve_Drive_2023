// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  PneumaticHub mHub;
  private static int forwardChannel;
  private static int reverseChannel;
  static DoubleSolenoid m_DoubleSolenoid;
  /** Creates a new Gripper. */
  public Gripper() {
    mHub = new PneumaticHub(Constants.PneumaticHubID);
    forwardChannel = 0;
    reverseChannel = 1;
    m_DoubleSolenoid = mHub.makeDoubleSolenoid(forwardChannel, reverseChannel);
  }

  public static void setForward(){
    m_DoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  public static void setReverse(){
    m_DoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
