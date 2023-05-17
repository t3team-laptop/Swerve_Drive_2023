// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Position;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.subsystems.Intake.Wrist;

public class HumanPlayerPreset extends CommandBase {
  private ElevatorPivot elevatorPivot;
  private Wrist wrist;
  /** Creates a new FloorPreset. */
  public HumanPlayerPreset(ElevatorPivot elevator, Wrist wrist) {
    this.elevatorPivot = elevator;
    addRequirements(elevator);
    this.wrist = wrist;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.rotateState = Position.HP;
    wrist.setPivotPosition(Position.HP.getWrist(), true);
    elevatorPivot.rotateState = Position.HP;
    elevatorPivot.setPivotPosition(Position.HP.getPivot(), true);
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
