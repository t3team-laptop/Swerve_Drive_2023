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
  private DoubleSolenoid mSolenoid;
  private boolean open = true;
  /** Creates a new Gripper. */
  public Gripper() {
    pneumaticHub = new PneumaticHub(Constants.PneumaticHubID);
    pneumaticHub.enableCompressorDigital();
    mSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
    System.out.println("Hub ID: " + Constants.PneumaticHubID);
    System.out.println(pneumaticHub);
    //System.out.println("Worked");
  }

  public void close(){
    mSolenoid.set(Value.kForward);
    open = false;
  }
  public void open(){
    mSolenoid.set(Value.kReverse);
    open = true;
  }
  @Override
  public void periodic() {
     // This method will be called once per scheduler run
     SmartDashboard.putBoolean("Gripper signal", open);
  }
}
