package frc.robot;

import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Position;
import frc.robot.Constants.LEDs.LEDMode;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Arm.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeModule;
import frc.robot.subsystems.Intake.Wrist;
import frc.robot.subsystems.LEDs.LEDs;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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
    private final int pivotAxis = XboxController.Axis.kLeftY.value;
    private final int extensionAxis = XboxController.Axis.kRightY.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(baseDriver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(baseDriver, 8);

    /* Subsystems */
    private final Vision s_Vision = new Vision();
    private final Swerve s_Swerve = new Swerve();
    private final ElevatorPivot elevatorPivot = new ElevatorPivot();
    private final LEDs leds = new LEDs();
    private final IntakeModule clawMotors = new IntakeModule();
    private final Wrist wrist = new Wrist();

    /* Autonomous Mode Chooser */
    private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

    /* Autonomous */
     
    private static Map<String, Command> eventMap = new HashMap<>();
    {
        eventMap.put("setcubelvl3", new MiddlePreset(elevatorPivot, wrist).withTimeout(2));
        eventMap.put("X-Lock", new XLock(s_Swerve));
        eventMap.put("Outtake", new Outtake(clawMotors).withTimeout(1));
        eventMap.put("Zero", new Zero(elevatorPivot, wrist).withTimeout(2));
        eventMap.put("armHigh", new ResetArm(elevatorPivot, wrist).withTimeout(2));
        eventMap.put("zeroGyro", new InstantCommand(() -> s_Swerve.zeroGyro()));
        eventMap.put("fixSwerve", new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
}

 

    private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose,
            s_Swerve::resetOdometry,
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(Constants.AutoConstants.kPXController, 0, 0),
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
            s_Swerve::setModuleStates, eventMap, true, s_Swerve);

    private final PathPlannerTrajectory leaveCommunityAndDock = PathPlanner.loadPath("Leave Community && Dock",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final PathPlannerTrajectory communityLeaveOnly = PathPlanner.loadPath("Community Leave Only",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final PathPlannerTrajectory allCommunityLvl3 = PathPlanner.loadPath("All Community Lvl 3",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final PathPlannerTrajectory twoCommunityLvl3 = PathPlanner.loadPath("2 Community Lvl 3",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final PathPlannerTrajectory leaveScoreDock = PathPlanner.loadPath("Leave, Score, Dock",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final PathPlannerTrajectory scoreLeftTwice = PathPlanner.loadPath("leftScoreTwice",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final PathPlannerTrajectory leftScoreTwiceAndCharge = PathPlanner.loadPath("leftScoreTwiceAndCharge",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final PathPlannerTrajectory MiddlePlaceAndCharge = PathPlanner.loadPath("MiddlePlaceAndCharge",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final PathPlannerTrajectory RightScoreTwo = PathPlanner.loadPath("RightScoreTwo",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final PathPlannerTrajectory justMoving = PathPlanner.loadPath("Just Moving",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    private final PathPlannerTrajectory MiddlePlace = PathPlanner.loadPath("Middle Place",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    private final PathPlannerTrajectory DONOTHING = PathPlanner.loadPath("DONOTHING",
             Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);


    // Commands //
    FloorPreset floor;
    MiddlePreset middle;
    TopPreset top;
    ResetArm resetArm;
    Zero zero;
    HumanPlayerPreset hpp;
    ManualPivot manualPivot;

    XLock xLock;

    AutoBalancing autoBalance;
    YellowLEDs yellowLEDs;
    PurpleLEDs purpleLEDs;
    
    Intake intake;
    Outtake outtake;
    

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        CameraServer.startAutomaticCapture();
           
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -baseDriver.getRawAxis(translationAxis),
                        () -> -baseDriver.getRawAxis(strafeAxis),
                        () -> -baseDriver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        xLock = new XLock(s_Swerve);
        xLock.addRequirements(s_Swerve);
        elevatorPivot.setDefaultCommand(
                new ManualPivot(
                        elevatorPivot,
                        () -> armDriver.getRawAxis(pivotAxis)));
   

        wrist.setDefaultCommand(
                new ManualWrist(
                        wrist, 
                        () -> armDriver.getRawAxis(extensionAxis)));

        floor = new FloorPreset(elevatorPivot, wrist);
        floor.addRequirements(elevatorPivot);
        middle = new MiddlePreset(elevatorPivot, wrist);
        middle.addRequirements(elevatorPivot);
        top = new TopPreset(elevatorPivot, wrist);
        top.addRequirements(elevatorPivot);
        resetArm = new ResetArm(elevatorPivot, wrist);
        resetArm.addRequirements(elevatorPivot);
        zero = new Zero(elevatorPivot, wrist);
        zero.addRequirements(elevatorPivot);
        zero.addRequirements(wrist);
        hpp = new HumanPlayerPreset(elevatorPivot, wrist);
        hpp.addRequirements(elevatorPivot);
        hpp.addRequirements(wrist);



        autoBalance = new AutoBalancing(s_Swerve, true);
        autoBalance.addRequirements(s_Swerve);

        yellowLEDs = new YellowLEDs(leds);
        yellowLEDs.addRequirements(leds);
        purpleLEDs = new PurpleLEDs(leds);
        purpleLEDs.addRequirements(leds);

        intake = new Intake(clawMotors);
        intake.addRequirements(clawMotors);
        outtake = new Outtake(clawMotors);
        outtake.addRequirements(clawMotors);
        // Declare Driver Controller Buttons
        DA = new JoystickButton(baseDriver, 1);
        DB = new JoystickButton(baseDriver, 2);
        DX = new JoystickButton(baseDriver, 3);
        DY = new JoystickButton(baseDriver, 4);
        DLB = new JoystickButton(baseDriver, 5);
        DRB = new JoystickButton(baseDriver, 6);
        DM1 = new JoystickButton(baseDriver, 7);
        DM2 = new JoystickButton(baseDriver, 8);

        // Declare Arm Controller Buttons
        AA = new JoystickButton(armDriver, 1);
        AB = new JoystickButton(armDriver, 2);
        AX = new JoystickButton(armDriver, 3);
        AY = new JoystickButton(armDriver, 4);
        ALB = new JoystickButton(armDriver, 5);
        ARB = new JoystickButton(armDriver, 6);
        AM1 = new JoystickButton(armDriver, 8);
        AM2 = new JoystickButton(armDriver, 10);

        // Configure the button bindings
        configureButtonBindings();
        configureSmartDashboard();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by   
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
            AA.onTrue(floor);
            AB.onTrue(hpp);
            AX.onTrue(middle);
            AY.onTrue(resetArm);
            ALB.whileTrue(intake);
            ARB.whileTrue(outtake);
            AM1.onTrue(zero);
            // AM1.onTrue(yellowLEDs);
            // AM2.onTrue(purpleLEDs);

            /* Driver Buttons */
            zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
            DX.onTrue(xLock);
            DM1.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
    }

    private void configureSmartDashboard() {
        autoChooser.addOption("Leave Community && Dock", leaveCommunityAndDock);
        autoChooser.addOption("Community Leave Only", communityLeaveOnly);
        autoChooser.addOption("All Community Lvl 3", allCommunityLvl3);
        autoChooser.addOption("2 Community Lvl 3", twoCommunityLvl3);
        autoChooser.addOption("Leave, Score, Dock", leaveScoreDock);
        autoChooser.addOption("leftScoreTwice", scoreLeftTwice);
        autoChooser.addOption("leftScoreTwiceAndCharge", leftScoreTwiceAndCharge);
        autoChooser.addOption("MiddlePlaceAndCharge", MiddlePlaceAndCharge);
        autoChooser.addOption("RightScoreTwo", RightScoreTwo);
        autoChooser.addOption("Just Moving", justMoving);
        autoChooser.addOption("Middle Place", MiddlePlace);
        autoChooser.addOption("DO NOTHING", DONOTHING);

        SmartDashboard.putData(autoChooser);
    }

    /**
         * Ran once the robot is put in disabled
         */
        public void disabledInit() {
            s_Swerve.resetModulesToAbsolute();
            leds.setLEDMode(LEDMode.WHITEDOT);
    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        s_Swerve.resetEverthing();
        if(autoChooser.getSelected() != MiddlePlaceAndCharge){
          return autoBuilder.fullAuto(autoChooser.getSelected());
        }
        return Commands.sequence(new MiddlePreset(elevatorPivot, wrist).withTimeout(2), 
        new Outtake(clawMotors).withTimeout(1), 
        new ResetArm(elevatorPivot, wrist).withTimeout(2), 
        autoBuilder.fullAuto(justMoving));
        //return doNothing;
    }
}
