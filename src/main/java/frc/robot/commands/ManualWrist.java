// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.subsystems.Intake.Wrist;

public class ManualWrist extends CommandBase {
  private Wrist wrist;
  private DoubleSupplier ySup;

  /** Creates a new ManualPivot. */
  public ManualWrist(Wrist wrist, DoubleSupplier ySup) {
    this.wrist = wrist;
    addRequirements(wrist);
    
    this.ySup = ySup;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yVal = MathUtil.applyDeadband(ySup.getAsDouble(), Constants.stickDeadband);
    //elevator.setPivotPosition(elevator.getEncoderPosition() + yVal*6000, false);
    wrist.setPivotPosition(yVal, false);
    //elevator.isPosition = false;
    //elevator.percentOutput = yVal;
  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setArmState(Position.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
