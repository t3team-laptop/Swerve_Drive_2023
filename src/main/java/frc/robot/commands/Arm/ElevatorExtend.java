// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorExtension;

public class ElevatorExtend extends CommandBase {
  private ElevatorExtension elevator;
  private DoubleSupplier axisSup;
  /** Creates a new ElevatorExtend. */
  public ElevatorExtend(ElevatorExtension elevator, DoubleSupplier axisSup) {
    this.elevator = elevator;
    addRequirements(elevator);

    this.axisSup = axisSup;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double axisVal = MathUtil.applyDeadband(axisSup.getAsDouble(), Constants.stickDeadband);
    if(axisVal > 0 && elevator.getExtensionMotor().getSelectedSensorPosition() < 100){ //TODO: find upper bound for elevator extension
    elevator.extend(axisVal);
    }
    else if(axisVal < 0 && elevator.getExtensionMotor().getSelectedSensorPosition() > 0){ //TODO: find lower bound for elevator extension
      elevator.retract(-axisVal);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopExtension();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}