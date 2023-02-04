package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Arm.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    JoystickButton DA, DB, DX, DY, DLB, DRB, DRT, DLT, DM1, DM2;
    JoystickButton AA, AB, AX, AY, ALB, ARB, ALT, ART, AM1, AM2;

    /* Controllers */
    private final XboxController baseDriver = new XboxController(0);
    private final XboxController armDriver = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Arm Controls */
    private final int yAxis = XboxController.Axis.kLeftY.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(baseDriver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(baseDriver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Elevator elevator;

    // Commands //
    CeilingPreset ceil;
    ElevatorExtend extend;
    RepeatCommand repeatExt;
    ElevatorRetract retract;
    RepeatCommand repeatRet;
    FloorPreset floor;
    MiddlePreset middle;
    TopPreset top;

    



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -baseDriver.getRawAxis(translationAxis), 
                () -> -baseDriver.getRawAxis(strafeAxis), 
                () -> -baseDriver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );


        elevator = new Elevator();

        elevator.setDefaultCommand(
            new ManualPivot(
                elevator,
                () -> armDriver.getRawAxis(yAxis)
            )   
        );

        ceil = new CeilingPreset(elevator);
        ceil.addRequirements(elevator);
        extend = new ElevatorExtend(elevator);
        extend.addRequirements(elevator);
        repeatExt = new RepeatCommand(extend);
        retract = new ElevatorRetract(elevator);
        retract.addRequirements(elevator);
        repeatRet = new RepeatCommand(retract);
        floor = new FloorPreset(elevator);
        floor.addRequirements(elevator);
        middle = new MiddlePreset(elevator);
        middle.addRequirements(elevator);
        top = new TopPreset(elevator);
        top.addRequirements(elevator);

        // Configure the button bindings
        configureButtonBindings();

         //Declare Driver Controller Buttons
         DA = new JoystickButton(baseDriver, 1);
         DB = new JoystickButton(baseDriver, 2);
         DX = new JoystickButton(baseDriver, 3);
         DY = new JoystickButton(baseDriver, 4);
         DLB = new JoystickButton(baseDriver, 5);
         DRB = new JoystickButton(baseDriver, 6);
         DLT = new JoystickButton(baseDriver, 2);
         DRT = new JoystickButton(baseDriver, 3);
         DM1 = new JoystickButton(baseDriver, 7);
         DM2 = new JoystickButton(baseDriver, 8);

         //Declare Arm Controller Buttons
         AA = new JoystickButton(armDriver, 1);
         AB = new JoystickButton(armDriver, 2);
         AX = new JoystickButton(armDriver, 3);
         AY = new JoystickButton(armDriver, 4);
         ALB = new JoystickButton(armDriver, 5);
         ARB = new JoystickButton(armDriver, 6);
         ALT = new JoystickButton(armDriver, 2);
         ART = new JoystickButton(armDriver, 3);
         AM1 = new JoystickButton(armDriver, 7);
         AM2 = new JoystickButton(armDriver, 8);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /*  Configure Driver Controller Buttons 
        ALB.whileTrue(repeatRet);
        ARB.whileTrue(repeatExt);
        AB.onTrue(ceil);
        AX.onTrue(floor);
        AY.onTrue(top);
        AA.onTrue(middle);
        */

        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
