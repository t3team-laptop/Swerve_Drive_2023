package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.hal.simulation.REVPHDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Gripper.CloseGripper;
import frc.robot.commands.Gripper.OpenGripper;
import frc.robot.subsystems.*;

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
    private final JoystickButton robotCentric = new JoystickButton(baseDriver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ElevatorPivot elevatorPivot = new ElevatorPivot();
    private final ElevatorExtension elevatorExtension = new ElevatorExtension();;
    private final Gripper gripper = new Gripper();

    /* Autonomous Mode Chooser */
    private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

    /* Autonomous */
    private static Map<String, Command> eventMap = new HashMap<>();
    {
        eventMap.put("setcubelvl3", new TopPreset(elevatorPivot, elevatorExtension));
        eventMap.put("releaseGripper", new OpenGripper(gripper));
        eventMap.put("autoBalance", new AutoBalancing(s_Swerve, true));
        eventMap.put("floorArm", new FloorPreset(elevatorPivot));
        eventMap.put("closeGripper", new CloseGripper(gripper));
        eventMap.put("setconelvl3", new TopPreset(elevatorPivot, elevatorExtension));
        eventMap.put("resetArm", new ResetArm(elevatorPivot, elevatorExtension));
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


    // Commands //
    FloorPreset floor;
    MiddlePreset middle;
    TopPreset top;
    ResetArm resetArm;

    CloseGripper closeGripper;
    OpenGripper openGripper;

    XLock xLock;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
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

        elevatorExtension.setDefaultCommand(
                new ElevatorExtend(
                        elevatorExtension,
                        () -> armDriver.getRawAxis(extensionAxis)));
        floor = new FloorPreset(elevatorPivot);
        floor.addRequirements(elevatorPivot);
        middle = new MiddlePreset(elevatorPivot, elevatorExtension);
        middle.addRequirements(elevatorPivot);
        middle.addRequirements(elevatorExtension);
        top = new TopPreset(elevatorPivot, elevatorExtension);
        top.addRequirements(elevatorPivot);
        top.addRequirements(elevatorExtension);
        resetArm = new ResetArm(elevatorPivot, elevatorExtension);
        resetArm.addRequirements(elevatorPivot);
        resetArm.addRequirements(elevatorExtension);


        closeGripper = new CloseGripper(gripper);
        closeGripper.addRequirements(gripper);
        openGripper = new OpenGripper(gripper);
        openGripper.addRequirements(gripper);

        // Declare Driver Controller Buttons
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

        // Declare Arm Controller Buttons
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
        AX.onTrue(floor);
        AY.onTrue(top);
        AA.onTrue(middle);
        AB.onTrue(resetArm);
        ALB.onTrue(closeGripper);
        ARB.onTrue(openGripper);
        
        DX.onTrue(xLock);

        /*  Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        DX.onTrue(xLock);
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



        SmartDashboard.putData(autoChooser);
    }

    /**
         * Ran once the robot is put in disabled
         */
        public void disabledInit() {
            s_Swerve.resetModulesToAbsolute();
    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoBuilder.fullAuto(autoChooser.getSelected());
    }
}
