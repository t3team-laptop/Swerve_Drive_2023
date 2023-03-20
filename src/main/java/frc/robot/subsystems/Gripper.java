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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  private PneumaticHub pneumaticHub;
  //private DoubleSolenoid mSolenoid;
  private boolean open = true;
  /** Creates a new Gripper. */
  public Gripper() {
    pneumaticHub = new PneumaticHub(Constants.PneumaticHubID);
    pneumaticHub.enableCompressorDigital();
    pneumaticHub.clearStickyFaults();
    pneumaticHub.makeDoubleSolenoid(0, 7);
   // mSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 7);
  }

  public void open(){
    pneumaticHub.setSolenoids(pneumaticHub.getSolenoids(), 1);
    open = false;
  }
  public void close(){
    pneumaticHub.setSolenoids(7, 1);
    open = true;
  }
  @Override
  public void periodic() {
     // This method will be called once per scheduler run
     SmartDashboard.putBoolean("Gripper signal", open);
  }
}
