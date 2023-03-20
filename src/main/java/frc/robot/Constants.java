package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.HSV;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

     // Elevator
    public static final int ElevatorPivotLeftID = 20;
    public static final int ElevatorPivotRightID = 21;
    public static final int ElevatorExtensionLeftID = 22;
    public static final int ElevatorExtensionRightID= 23;

      // Pivot set points
    public static final double kPivotGroundCount = 0;
    public static final double kPivotScoreCount = -55;
    public static final double kPivotPreScoreCount = -65;
    public static final double kPivotStowCount = -120;
    public static final int kTimeoutMs = 0;

    // Extension set points
    public static final double kExtensionStowCount = 0;
    public static final double kExtensionMidGoalCount = 34;
    public static final double kExtensionHighGoalCount = 72;

    // Gripper 
    public static final int PneumaticHubID = 11;

    public static final class Swerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18); //TODO: This must be tuned to specific robot but already set for wooden swerve
        public static final double wheelBase = Units.inchesToMeters(29.5); //TODO: This must be tuned to specific robot but already set for wooden swerve
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double pitchSetPoint = 0.0;

        public static final double drivePitchKP = 0.04;
        public static final double drivePitchKI = 0.00005;
        public static final double drivePitchKD = 0.000000000000001;
        public static final double drivePitchKFF = 0.000000000000001;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = 0.078083; //TODO: This must be tuned to specific robot
        public static final double driveKV = 0.00036451;
        public static final double driveKA = 3.9469E-05;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.8; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Swerve Limiting Values */
        public static final double autoCenterLimit = .3;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(2.197266);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
          public static final class Mod1 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 2;
          public static final int angleMotorID = 3;
          public static final int canCoderID = 1;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(116.630859);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(192.744141);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 6;
          public static final int angleMotorID = 7;
          public static final int canCoderID = 3;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(148.447266);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
      public static final PathConstraints constraints = new PathConstraints(1, 1);
  
      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;
    }
  
    public static final class PhotonVision{
      public static final String photonVisionName = "OV5647";
      public static final Transform3d robotToCam =
      new Transform3d(
              new Translation3d(Units.inchesToMeters(11.4), 0.0, Units.inchesToMeters(6.4)),
              new Rotation3d(
                      0, 0,
                      0));
    }
    public enum Position {

      HIGH(0, 0),
      CONEHIGH(.134327, 35),
      CUBEHIGH(1.55, 35),
      MID(0, 0),
      CONEMID(.104327, 23),
      CUBEMID(1.427, 16.5),
      FLOOR(.5236, .25),
      STANDBY(1.1765, .25),
      CUBEINTAKE(0.0, 0.3),
      STANDINGCONEINTAKE(5.106, 14.380),
      TIPPEDCONEINTAKE(5.572, 1.333),
      HUMANPLAYERINTAKE(.8763, 1.5);

      private double pivotPos;
        private double extendPos;

        private Position(double pivot, double extend) {
            this.pivotPos = pivot;
            this.extendPos = extend;
        }

        public double getPivot() {
            return pivotPos;
        }

        public double getExtend() {
            return extendPos;
        }
    }
    public enum GamePiece {
      CUBE(1),
      CONE(-1);

      private double direction;

      private GamePiece(double value) {
          direction = value;
      }

      public double getDirection() {
          return direction;
      }
  }
  
    public static final class AprilTags {
      public static final AprilTag tag1 = new AprilTag(1, FieldConstants.aprilTags.get(1));
      public static final AprilTag tag2 = new AprilTag(2, FieldConstants.aprilTags.get(2));
      public static final AprilTag tag3 = new AprilTag(3, FieldConstants.aprilTags.get(3));
      public static final AprilTag tag4 = new AprilTag(4, FieldConstants.aprilTags.get(4));
      public static final AprilTag tag5 = new AprilTag(5, FieldConstants.aprilTags.get(5));
      public static final AprilTag tag6 = new AprilTag(6, FieldConstants.aprilTags.get(6));
      public static final AprilTag tag7 = new AprilTag(7, FieldConstants.aprilTags.get(7));
      public static final AprilTag tag8 = new AprilTag(8, FieldConstants.aprilTags.get(8));
      public static final ArrayList<AprilTag> aprilTagList = new ArrayList<>();
  
      static {
        aprilTagList.add(tag1);
        aprilTagList.add(tag2);
        aprilTagList.add(tag3);
        aprilTagList.add(tag4);
        aprilTagList.add(tag5);
        aprilTagList.add(tag6);
        aprilTagList.add(tag7);
        aprilTagList.add(tag8);
      }
    }

    public enum SEGMENT { // Numbers in order of segment from left to right (driver station POV)
        CONE_1(0), CONE_2(1), CONE_3(3), CONE_4(-1), CONE_5(-1), CONE_6(-1),
        CUBE_1(0), CUBE_2(1), CUBE_3(3);
    
        private int level;
    
        private SEGMENT(int level){
          this.level = level;
        }
    
        public static SEGMENT getSegment(int level, boolean cone){
          if(cone){
            switch(level){
                case 1: return CONE_1;
                case 2: return CONE_2;
                case 3: return CONE_3;
            }
          }
          else{
            switch(level){
                case 1: return CUBE_1;
                case 2: return CUBE_2;
                case 3: return CUBE_3;
            }
          }
          return null;
        }
    
        public int getValue(){
          return level;
        }
        
      }
      public static final class LEDs {
        public static final int id = 9;
        public static final int length = 18;

        public static final LEDMode defaultMode = LEDMode.WHITEDOT;

        public static final class Flash {
            public static final double speed = 5;
        }
        public static final class WhiteDot{
          public static final HSV hsv = HSV.googleColorPickerHSV(44, 0, 100);

          public static final int pauseBetween = 0;
          public static final int length = 0;
          public static final double spread = 2;
          public static final double speed = .5;
        }
        public enum LEDMode {
            WHITEDOT("White Dot"),
            PURPLEFLASH("Purple Flash"),
            YELLOWFLASH("Yellow Flash");

            private String name;

            private LEDMode(String name) {
                this.name = name;
            }

            @Override
            public String toString() {
                return name;
            }
        }
  }
}