package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CanId;
import frc.robot.Constants.PidConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Commands.swerve.SwerveCommand;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class SwerveSubsystem {

    public static final CANBus kCANBus = new CANBus("Canivore", "./logs/example.hoot");

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(CanId.Pigeon)
            .withPigeon2Configs(SwerveConstants.pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(SwerveConstants.kDriveGearRatio)
            .withSteerMotorGearRatio(SwerveConstants.kSteerGearRatio)
            .withCouplingGearRatio(SwerveConstants.kCoupleRatio)
            .withWheelRadius(SwerveConstants.kWheelRadius)
            .withSteerMotorGains(PidConstants.steerGains)
            .withDriveMotorGains(PidConstants.driveGains)
            .withSteerMotorClosedLoopOutput(SwerveConstants.kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(SwerveConstants.kDriveClosedLoopOutput)
            .withSlipCurrent(SwerveConstants.kSlipCurrent)
            .withSpeedAt12Volts(SwerveConstants.kSpeedAt12Volts)
            .withDriveMotorType(SwerveConstants.kDriveMotorType)
            .withSteerMotorType(SwerveConstants.kSteerMotorType)
            .withFeedbackSource(SwerveConstants.kSteerFeedbackType)
            .withDriveMotorInitialConfigs(SwerveConstants.driveInitialConfigs)
            .withSteerMotorInitialConfigs(SwerveConstants.steerInitialConfigs)
            .withEncoderInitialConfigs(SwerveConstants.encoderInitialConfigs)
            .withSteerInertia(SwerveConstants.kSteerInertia)
            .withDriveInertia(SwerveConstants.kDriveInertia)
            .withSteerFrictionVoltage(SwerveConstants.kSteerFrictionVoltage)
            .withDriveFrictionVoltage(SwerveConstants.kDriveFrictionVoltage);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            CanId.Front_Left_Turn, CanId.Front_Left_Drive, CanId.Front_Left_CANcoder, SwerveConstants.kFrontLeftEncoderOffset,
            SwerveConstants.kFrontLeftXPos, SwerveConstants.kFrontLeftYPos, SwerveConstants.kInvertLeftSide, SwerveConstants.kFrontLeftSteerMotorInverted, Constants.SwerveConstants.kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            CanId.Front_Right_Turn, CanId.Front_Right_Drive, CanId.Front_Right_CANcoder, SwerveConstants.kFrontRightEncoderOffset,
            SwerveConstants.kFrontRightXPos, SwerveConstants.kFrontRightYPos, SwerveConstants.kInvertRightSide, SwerveConstants.kFrontRightSteerMotorInverted, Constants.SwerveConstants.kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            CanId.Back_Left_Turn, CanId.Back_Left_Drive, CanId.Back_Left_CANcoder, SwerveConstants.kBackLeftEncoderOffset,
            SwerveConstants.kBackLeftXPos, SwerveConstants.kBackLeftYPos, SwerveConstants.kInvertLeftSide, SwerveConstants.kBackLeftSteerMotorInverted, Constants.SwerveConstants.kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            Constants.CanId.Back_Right_Turn, CanId.Back_Right_Drive, CanId.Back_Right_CANcoder, SwerveConstants.kBackRightEncoderOffset,
            SwerveConstants.kBackRightXPos, SwerveConstants.kBackRightYPos, SwerveConstants.kInvertRightSide, SwerveConstants.kBackRightSteerMotorInverted, Constants.SwerveConstants.kBackRightEncoderInverted
        );

    public static SwerveCommand createDrivetrain() {
        return new SwerveCommand(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }

    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
       
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }

        private final SwerveModule<TalonFX, TalonFX, CANcoder> frontLeft = getModule(0);
        private final SwerveModule<TalonFX, TalonFX, CANcoder> frontRight = getModule(1);
        private final SwerveModule<TalonFX, TalonFX, CANcoder> backLeft = getModule(2);
        private final SwerveModule<TalonFX, TalonFX, CANcoder>backRight = getModule(3);

        private final TalonFX frontLeftDrive = frontLeft.getDriveMotor();
        private final TalonFX frontRightDrive = frontRight.getDriveMotor();
        private final TalonFX backLeftDrive = backLeft.getDriveMotor();
        private final TalonFX backRightDrive = backRight.getDriveMotor();

        // private final TalonFX frontLeftSteer = frontLeft.getSteerMotor();
        // private final TalonFX frontRightSteer = frontLeft.getSteerMotor();
        // private final TalonFX backLeftSteer = backLeft.getSteerMotor();
        // private final TalonFX backRightSteer = backRight.getSteerMotor();

        private final CANcoder frontLeftEncoder = (CANcoder) frontLeft.getEncoder();
        private final CANcoder frontRightEncoder = (CANcoder) frontRight.getEncoder();
        private final CANcoder backLeftEncoder = (CANcoder) backLeft.getEncoder();
        private final CANcoder backRightEncoder = (CANcoder) backRight.getEncoder();

        private final Pigeon2 pigeon = new Pigeon2(CanId.Pigeon, "Canivore");

        private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
            SwerveConstants.kDriveKinematics,
            pigeon.getRotation2d(), 
            new SwerveModulePosition[]{
                new SwerveModulePosition(frontLeftDrive.getPosition().getValueAsDouble(), new Rotation2d(frontLeftEncoder.getAbsolutePosition().getValue())),
                new SwerveModulePosition(frontRightDrive.getPosition().getValueAsDouble(), new Rotation2d(frontRightEncoder.getAbsolutePosition().getValue())),
                new SwerveModulePosition(backLeftDrive.getPosition().getValueAsDouble(), new Rotation2d(backLeftEncoder.getAbsolutePosition().getValue())),
                new SwerveModulePosition(backRightDrive.getPosition().getValueAsDouble(), new Rotation2d(backRightEncoder.getAbsolutePosition().getValue()))
            }, new Pose2d(0, 0, new Rotation2d()) // Initial pose
            );

        public SwerveDriveOdometry getOdometer(){
            return odometer;
        }

        public Pose2d getPose2d(){
            return odometer.getPoseMeters();
            //Pose2d startPose = odometer.getPoseMeters(); // Your desired starting pose
            //Supplier<Pose2d> poseSupplier = () -> startPose;
            //return poseSupplier;
        }

        public void resetOdometer(Pose2d pose) {
            odometer.resetPosition(
                pigeon.getRotation2d(),
                new SwerveModulePosition[]{
                    new SwerveModulePosition(frontLeftDrive.getPosition().getValueAsDouble(), new Rotation2d(frontLeftEncoder.getAbsolutePosition().getValue())),
                    new SwerveModulePosition(frontRightDrive.getPosition().getValueAsDouble(), new Rotation2d(frontRightEncoder.getAbsolutePosition().getValue())),
                    new SwerveModulePosition(backLeftDrive.getPosition().getValueAsDouble(), new Rotation2d(backLeftEncoder.getAbsolutePosition().getValue())),
                    new SwerveModulePosition(backRightDrive.getPosition().getValueAsDouble(), new Rotation2d(backRightEncoder.getAbsolutePosition().getValue()))
                },
                pose
            );
        }

        public ChassisSpeeds getRobotRelativeSpeeds() {
            return getState().Speeds; 
        }
    }

        

}