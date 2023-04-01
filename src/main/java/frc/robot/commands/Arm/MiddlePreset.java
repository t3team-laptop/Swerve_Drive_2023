// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.ElevatorExtension;
import frc.robot.subsystems.ElevatorPivot;

public class MiddlePreset extends CommandBase {
  private ElevatorPivot elevatorPivot;
  private ElevatorExtension elevatorExtension;

  private boolean done = false;
  /** Creates a new FloorPreset. */
  public MiddlePreset(ElevatorPivot elevator, ElevatorExtension elevatorExtension) {
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
    elevatorPivot.rotateState = Position.CONEMID;
    elevatorPivot.setPivotPosition(Position.CONEMID.getPivot(), true);
    elevatorExtension.setExtensionPosition(Position.CONEMID.getExtend(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //elevator.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((elevatorPivot.getEncoderPosition() <= Position.CONEMID.getPivot()+5 && 
    elevatorPivot.getEncoderPosition() >= Position.CONEMID.getPivot()-5 ) &&
    ((elevatorExtension.getEncoderPosition() <= Position.CONEMID.getExtend()+5 && 
    elevatorExtension.getEncoderPosition() >= Position.CONEMID.getExtend()-5))){
      return true;
    }
    return false;
  }
}
