// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.subsystems.ElevatorExtension;

public class TopPreset extends CommandBase {
  private ElevatorPivot elevator;
  private ElevatorExtension elevatorExtension;
  /** Creates a new TopPreset. */
  public TopPreset(ElevatorPivot elevator, ElevatorExtension elevatorExtension) {
    this.elevator = elevator;
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
    elevator.goToPivotScore();
    elevatorExtension.goToExtensionHighGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
