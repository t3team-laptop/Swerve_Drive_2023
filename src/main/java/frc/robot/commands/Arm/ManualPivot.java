// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.ElevatorPivot;

public class ManualPivot extends CommandBase {
  private ElevatorPivot elevator;
  private DoubleSupplier ySup;

  /** Creates a new ManualPivot. */
  public ManualPivot(ElevatorPivot elevator, DoubleSupplier ySup) {
    this.elevator = elevator;
    addRequirements(elevator);
    
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
    elevator.setPivotPosition(yVal*2, false);
    //elevator.isPosition = false;
    //elevator.percentOutput = yVal;
  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setArmState(Position.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
