package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;

public class WaypointConstants {
    // TODO: the acceleration is defently off so we need to find out the true value
    private static final PathConstraints constraints = new PathConstraints(
        DrivetrainConstants.maxSpeedMetersPerSecond, DrivetrainConstants.maxSpeedMetersPerSecond,
        DrivetrainConstants.maxAngularVelocityRadiansPerSecond, DrivetrainConstants.maxAngularVelocityRadiansPerSecond);

    private static final Pose2d nearDepotPose = new Pose2d(2,6,Rotation2d.fromDegrees(0));
    
    public static final Command goNearDepot = AutoBuilder.pathfindToPose(nearDepotPose,constraints);

}
