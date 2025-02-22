package frc.robot.commands.swerve;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.MathUtils;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoLineUpReef extends SequentialCommandGroup {

    private CommandSwerveDrivetrain swerve;

    public AutoLineUpReef(CommandSwerveDrivetrain swerve, int right) {

        this.swerve = swerve;

        addRequirements(this.swerve);

        // Add conditional logic to choose commands
        addCommands(
            new DeferredCommand(
                () -> (new ConditionalAllianceCommand(
                    new CloseDriveToPose(swerve, addRCtoFC(MathUtils.findClosestTarget(this.swerve.getState().Pose, Constants.AutoDriveConstants.BLUE_REEF_POSES), right)),
                    new CloseDriveToPose(swerve, addRCtoFC(MathUtils.findClosestTarget(this.swerve.getState().Pose, Constants.AutoDriveConstants.RED_REEF_POSES), right))
                )),
                getRequirements()
            )
        );
    }

    public Pose2d addRCtoFC(Pose2d robotPose, int right) {
        double xOffset = Constants.AutoDriveConstants.ADDITIONS[right][0];
        double yOffset = Constants.AutoDriveConstants.ADDITIONS[right][1];

        double newX = robotPose.getX() + (xOffset * Math.cos(robotPose.getRotation().getRadians()) - yOffset * Math.sin(robotPose.getRotation().getRadians()));
        double newY = robotPose.getY() + (xOffset * Math.sin(robotPose.getRotation().getRadians()) + yOffset * Math.cos(robotPose.getRotation().getRadians()));
        Pose2d calculated = new Pose2d(newX, newY, robotPose.getRotation());
        return calculated;
    }
}