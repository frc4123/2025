// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Commands.autos.MiddleCoral;
import frc.robot.Commands.swerve.SwerveCommand;
import frc.robot.Subsystems.SwerveSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;



public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.kMaxSpeed * 0.1).withRotationalDeadband(SwerveConstants.kMaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(SwerveConstants.kMaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(Constants.InputConstants.kDriverControllerPort0);

    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
    private final SwerveCommand drivetrain = SwerveSubsystem.createDrivetrain();
    

    private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

    public RobotContainer() {
        configureBindings();
        initializeAutoChooser();
    }

    private void configureBindings() {
        // X is forward - WPILIB
        // Y is left - WPILIB
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * SwerveConstants.kMaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * SwerveConstants.kMaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * SwerveConstants.kMaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.a().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.a().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.b().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.b().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    

    public void initializeAutoChooser(){
        m_autoChooser.setDefaultOption(
      "middleCoral", new WaitCommand(0.1)
      .andThen(new MiddleCoral(drivetrain).middleCoral())
      );

        SmartDashboard.putData("Auto Selector", m_autoChooser);
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
