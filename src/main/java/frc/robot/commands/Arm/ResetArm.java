// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

//import org.apache.commons.io.filefilter.TrueFileFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.ElevatorExtension;
import frc.robot.subsystems.ElevatorPivot;

public class ResetArm extends CommandBase {
  private ElevatorPivot elevatorPivot;
  private ElevatorExtension elevatorExtension;
  /** Creates a new FloorPreset. */
  public ResetArm(ElevatorPivot elevator, ElevatorExtension elevatorExtension) {
    this.elevatorPivot = elevator;
    addRequirements(elevator);
    this.elevatorExtension = elevatorExtension;
    addRequirements(elevatorExtension);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorPivot.setPivotPosition(0, true);
    elevatorExtension.setExtensionPosition(0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //elevator.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
