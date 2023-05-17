package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeModule extends SubsystemBase {
  // Motor for the intake mechanism
  private final CANSparkMax intakeMotor;
  private final double intakePower = 1;
  private final double outtakePower = -1;
  private boolean running;
  private static double tooHotTemperatureHigh = 80.0;
  private static final double tooHotTemperatureLow = 65.0;
  private boolean tooHotAlertActive = false;
  private final String tooHotAlert = "Intake motor disabled due to very high temperature.";
  public double motorTemperautre;


  // Constructor for initializing the intake module
  public IntakeModule() {
    intakeMotor = new CANSparkMax(Constants.intakeRollerPort, MotorType.kBrushless);
    intakeMotor.setSmartCurrentLimit(40, 30);
    motorTemperautre = intakeMotor.getMotorTemperature();
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
    setIntakePower(0);
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  

  @Override
  public void periodic(){
    motorTemperautre = intakeMotor.getMotorTemperature();
    if(tooHotAlertActive){
      intakeMotor.setVoltage(0.0);
      System.out.println(tooHotAlert);
    }
    // Update too hot alert
    if(motorTemperautre >= tooHotTemperatureHigh){tooHotAlertActive = true;}
    if (motorTemperautre < tooHotTemperatureHigh) {
      tooHotAlertActive = false;
    }
    if (tooHotAlertActive) {
      if (motorTemperautre < tooHotTemperatureLow) {
        tooHotAlertActive = false;
      }
    } 
  }
}