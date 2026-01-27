package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;

public class WaypointConstants {
    public static final Pose2d nearDepotPose = new Pose2d(2,6,Rotation2d.fromDegrees(55));
    public static final Pose2d nearOutpost = new Pose2d(2.5,2,Rotation2d.fromDegrees(116));
    public static final Pose2d nearHub = new Pose2d(3,3.5,Rotation2d.fromDegrees(135));

}
