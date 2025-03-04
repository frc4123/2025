package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    
    public static final class CanIdCanivore { 

        public static final int Front_Left_Drive = 2;
        public static final int Front_Right_Drive = 3;
        public static final int Back_Left_Drive = 4;
        public static final int Back_Right_Drive = 5;
        // drive motors - order - start top left in clockwise rotation

        public static final int Front_Left_Turn = 6;
        public static final int Front_Right_Turn = 7;
        public static final int Back_Left_Turn = 8;
        public static final int Back_Right_Turn = 9;
        // turn motors - order - start top left in clockwise rotation

        public static final int Pigeon = 10;

        public static final int Front_Left_CANcoder = 11;
        public static final int Front_Right_CANcoder = 12;
        public static final int Back_Left_CANcoder = 13;
        public static final int Back_Right_CANcoder = 14;

        public static final int Elevator = 15;

        public static final int Algae_Arm = 17;
    }

    public static final class CanIdRio{
        public static final int Elevator_CANDi = 16; 

        public static final int Algae_Intake = 18;
        public static final int Algae_CANDi = 19;
        public static final int Algae_CANRange = 20;

        public static final int Coral_Intake = 21;
        public static final int Coral_CANRange = 22;
    }

    public static final class InputConstants {
        public static final int kDriverControllerPort0 = 0;
        public static final int kDriverControllerPort1 = 1;
        public static final int kDriverControllerPort2 = 2;
        public static final boolean fieldOrientation = true;
        public static final double kDeadband = 0.028;
    }

    public static final class Arm {
        public static final double kP = 210;
        public static final double kI = 0;
        public static final double kD = 17;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 4;

        public static final double velocity = 1.8; // 0.75
        public static final double acceleration = 1.7; //0.55
        public static final double jerk = 3; // was 0.8

        public static final double gearRatio = 96.66;

        public static final double stowPosition = 0.34;
        public static final double upPosition = 0.30; // tune
        public static final double bargePosition = 0.22; //tune
        public static final double reefPosition = 0.125; // was 0.15
        public static final double processorPosition = 0; 
        public static final double outPosition = -0.035;
    }

    public static final class Elevator {
        public static final double kP = 85;
        public static final double kI = 0;
        public static final double kD = 5;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 16;

        public static final double velocity = 72; //120; // max is 290 (don't go 290)
        public static final double acceleration = 225; //100;
        public static final double jerk = 380; //80; 

        public static final double gearRatio = -1;

        public static final double down = 0;
        public static final double algaeGround = 16; // tune
        public static final double l1 = 24;
        public static final double l2Algae = 27;
        public static final double l2 = 30;
        public static final double l3 = 50;
        public static final double l4 = 78.5; // tune
        public static final double up = 88; //technically its 88.4
    }

    public static final class SwerveConstants {

        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;

        public static final double kWheelBase = 24.75;
        public static final double kTrackWidth = Units.inchesToMeters(24.75);

        //Front Left
        public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.427734375);
        public static final boolean kFrontLeftSteerMotorInverted = false;
        public static final boolean kFrontLeftEncoderInverted = false;

        public static final Distance kFrontLeftXPos = Inches.of(12.375);
        public static final Distance kFrontLeftYPos = Inches.of(12.375);

        //Front Right
        public static final Angle kFrontRightEncoderOffset = Rotations.of(-0.444580078125);
        public static final boolean kFrontRightSteerMotorInverted = false;
        public static final boolean kFrontRightEncoderInverted = false;

        public static final Distance kFrontRightXPos = Inches.of(12.375);
        public static final Distance kFrontRightYPos = Inches.of(-12.375);

        //Back Left
        public static final Angle kBackLeftEncoderOffset = Rotations.of(0.3447265625);
        public static final boolean kBackLeftSteerMotorInverted = false;
        public static final boolean kBackLeftEncoderInverted = false;
    
        public static final Distance kBackLeftXPos = Inches.of(-12.375);
        public static final Distance kBackLeftYPos = Inches.of(12.375);

        //Back Right
        public static final Angle kBackRightEncoderOffset = Rotations.of(-0.191162109375);
        public static final boolean kBackRightSteerMotorInverted = false;
        public static final boolean kBackRightEncoderInverted = false;

        public static final Distance kBackRightXPos = Inches.of(-12.375);
        public static final Distance kBackRightYPos = Inches.of(-12.375);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // front right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // back left
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back right

        public static final double CLOSE_TRANSLATION_PP_KP = 6; // 8
        public static final double CLOSE_TRANSLATION_PP_KI = 0.0;
        public static final double CLOSE_TRANSLATION_PP_KD = 0.0;

        public static final double CLOSE_ROTATION_PP_KP = 5; // 8
        public static final double CLOSE_ROTATION_PP_KI = 0.0;
        public static final double CLOSE_ROTATION_PP_KD = 0.0;
    }

    public static class AutoDriveConstants {

        public static final Pose2d[] BLUE_REEF_POSES;
        public static final Pose2d[] RED_REEF_POSES;
    
        static { // Use a static block to initialize static variables
            if (DriverStation.isDSAttached() && DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                BLUE_REEF_POSES = new Pose2d[]{
                    new Pose2d(3.6576, 4.0259, new Rotation2d(0)),
                    new Pose2d(4.073906, 3.306318, new Rotation2d(Math.toRadians(60))),
                    new Pose2d(4.90474, 3.306318, new Rotation2d(Math.toRadians(120))),
                    new Pose2d(5.321046, 4.0259, new Rotation2d(Math.toRadians(180))),
                    new Pose2d(4.904739999999999, 4.745482, new Rotation2d(Math.toRadians(-120))),
                    new Pose2d(4.073906, 4.745482, new Rotation2d(Math.toRadians(-60)))
                };
    
                RED_REEF_POSES = new Pose2d[]{
                    new Pose2d(13.890498, 4.0259, new Rotation2d(0)),
                    new Pose2d(13.474446, 4.745482, new Rotation2d(Math.toRadians(60))),
                    new Pose2d(12.643358, 4.745482, new Rotation2d(Math.toRadians(120))),
                    new Pose2d(12.227305999999999, 4.0259, new Rotation2d(Math.toRadians(180))),
                    new Pose2d(12.643358, 3.3063179999999996, new Rotation2d(Math.toRadians(-120))),
                    new Pose2d(13.474446, 3.3063179999999996, new Rotation2d(Math.toRadians(-60)))
                };
            } else {
                BLUE_REEF_POSES = new Pose2d[]{
                    new Pose2d(13.890498, 4.0259, new Rotation2d(0)),
                    new Pose2d(13.474446, 4.745482, new Rotation2d(Math.toRadians(60))),
                    new Pose2d(12.643358, 4.745482, new Rotation2d(Math.toRadians(120))),
                    new Pose2d(12.227305999999999, 4.0259, new Rotation2d(Math.toRadians(180))),
                    new Pose2d(12.643358, 3.3063179999999996, new Rotation2d(Math.toRadians(-120))),
                    new Pose2d(13.474446, 3.3063179999999996, new Rotation2d(Math.toRadians(-60)))
                };
    
                RED_REEF_POSES = new Pose2d[]{
                    new Pose2d(3.6576, 4.0259, new Rotation2d(0)),
                    new Pose2d(4.073906, 3.306318, new Rotation2d(Math.toRadians(60))),
                    new Pose2d(4.90474, 3.306318, new Rotation2d(Math.toRadians(120))),
                    new Pose2d(5.321046, 4.0259, new Rotation2d(Math.toRadians(180))),
                    new Pose2d(4.904739999999999, 4.745482, new Rotation2d(Math.toRadians(-120))),
                    new Pose2d(4.073906, 4.745482, new Rotation2d(Math.toRadians(-60)))
                };
            }
        }
    
        public static final double[][] ADDITIONS = {
            {-0.4762, -0.075}, // LEFT ADDITION
            {-0.4762, -0.47}  // RIGHT ADDITION
        };
    }
    


public class MathUtils {

    public static boolean withinTolerance(Rotation2d value, Rotation2d tolerance) {
        return withinTolerance(value.getDegrees(), tolerance.getDegrees());
    }

    public static boolean withinTolerance(double value, double tolerance) {
        return Math.abs(value) <= Math.abs(tolerance);
    }

    public static Pose2d findClosestTarget(Pose2d current, Pose2d[] targets) {
        if (current == null) {
            return null;
        }
        if (targets == null) {
            throw new IllegalArgumentException("Target list cannot be null or empty.");
        }

        Pose2d closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Pose2d target : targets) {
            double distance = current.getTranslation().getDistance(target.getTranslation());
            if (distance <= minDistance) {
                minDistance = distance;
                closest = target;
            }
        }

        return closest;
    }

    
}

    public static final class VisionConstants{
        //Front Forward Camera Translation and Angle
        public static final double frontX = Units.inchesToMeters(7.176364); // 7.495 7.176364 -7.176364
        public static final double frontY = Units.inchesToMeters(7.495000); // -7.176364 7.495000 -7.495000
        public static final double frontZ = Units.inchesToMeters(6.857670); // 7.02

        public static final double frontRoll = Math.toRadians(0);
        public static final double frontPitch = Math.toRadians(-25); // 25
        public static final double frontYaw = Math.toRadians(0);

        //Front Angled Camera Translation and Angle
        public static final double angledX = Units.inchesToMeters(13.351494); // -11.7 12.75
        public static final double angledY = Units.inchesToMeters(2.766400); // 1.900142 3
        public static final double angledZ = Units.inchesToMeters(9.240711);

        public static final double angledRoll = Math.toRadians(0);
        public static final double angledPitch = Math.toRadians(-10); // 10
        public static final double angledYaw = Math.toRadians(55); // -55
    }
}
