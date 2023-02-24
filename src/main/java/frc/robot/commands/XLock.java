// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public class XLock extends CommandBase {
  private Swerve s_Swerve;
  /** Creates a new XLock. */
  public XLock(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.mSwerveMods[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)),true);
    s_Swerve.mSwerveMods[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315.0)),true);
    s_Swerve.mSwerveMods[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315.0)),true);
    s_Swerve.mSwerveMods[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)),true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
