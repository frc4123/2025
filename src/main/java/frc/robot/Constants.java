package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
    
    public static final class CanId { 

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
    }

    public static final class InputConstants {
        public static final int kDriverControllerPort0 = 0;
        public static final int kDriverControllerPort1 = 1;
        public static final int kDriverControllerPort2 = 2;
        public static final boolean fieldOrientation = true;
        public static final double kDeadband = 0.028;
    }

    public static final class PidConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        public static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(1.59).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        public static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);
    }

    public static final class SwerveConstants {

        public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
        public static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        public static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            );
        public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        public static final Pigeon2Configuration pigeonConfigs = null;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        public static final Current kSlipCurrent = Amps.of(120.0);

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(10.82);

        // used for simulation
        public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);

        // Simulated voltage necessary to overcome friction
        public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final double kMaxSpeed = kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;

        public static final double kCoupleRatio = 0;
        
        public static final double kWheelBase = 24.75;
        public static final double kTrackWidth = Units.inchesToMeters(24.75);

        public static final double kDriveGearRatio = 5.901;
        public static final double kSteerGearRatio = 12.8;
        public static final Distance kWheelRadius = Inches.of(4);

            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // front right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // back left
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back right

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
    }
}
