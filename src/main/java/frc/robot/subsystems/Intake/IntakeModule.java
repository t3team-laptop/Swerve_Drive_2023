package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeModule extends SubsystemBase {
  // Motor for the intake mechanism
  private final CANSparkMax intakeMotor;
  private final double intakePower = .5;
  private final double outtakePower = -.1;
  private boolean running;

  // Constructor for initializing the intake module
  public IntakeModule() {
    intakeMotor = new CANSparkMax(Constants.intakeRollerPort, MotorType.kBrushless);
    intakeMotor.setSmartCurrentLimit(10, 5);
  }

  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  public void intake(){
    running = true;
    setIntakePower(intakePower);
  }

  public void outtake(){ 
    running = true;
    setIntakePower(outtakePower);
  }

  public void stop(){
    intakeMotor.setIdleMode(IdleMode.kBrake);
    setIntakePower(0);
  }

}