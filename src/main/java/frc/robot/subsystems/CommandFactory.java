package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.WaypointConstants;
import java.util.Set;
import java.util.function.Supplier;

public class CommandFactory {

  private CommandSwerveDrivetrain m_drivetrain;
  private int m_side = 1;
  private Turret m_turret;
  private Zones m_zone;

  public CommandFactory(CommandSwerveDrivetrain drivetrain, Turret turret, Zones zone) {
    this.m_drivetrain = drivetrain;
    this.m_turret = turret;
    this.m_zone = zone;
  }

  public Command driveCircle() {
    return Commands.defer(
            () -> {
              switch (m_side) {
                case 1:
                  System.out.println("Heading To BottomLeftCorner");
                  return m_drivetrain
                      .goToPose(WaypointConstants.BottomLeftCornner)
                      .andThen(() -> m_side = 2);

                case 2:
                  System.out.println("Heading To TopLeftCorner");
                  return m_drivetrain
                      .goToPose(WaypointConstants.TopLeftCornner)
                      .andThen(() -> m_side = 3);

                case 3:
                  System.out.println("Heading To TopRightCorner");
                  return m_drivetrain
                      .goToPose(WaypointConstants.TopRightCornner)
                      .andThen(() -> m_side = 4);

                case 4:
                default:
                  System.out.println("Heading To BottomRightCorner");
                  return m_drivetrain
                      .goToPose(WaypointConstants.BottomRightCornner)
                      .andThen(() -> m_side = 1);
              }
            },
            Set.of(m_drivetrain))
        .repeatedly();
  }

  public Command turretTrackPredictedPositionCommand() {
    Supplier<Double> distance = () -> m_turret.getDistanceFromHub(m_drivetrain::getPose2d, m_zone);
    Supplier<Double> time = () -> m_turret.getTimeFromDistance(distance);
    Supplier<Pose2d> predictPose2d = () -> m_turret.calculatePredictedPose2d(m_drivetrain, time);
    return Commands.run(
        () -> {
          m_turret.trackTargetPose(predictPose2d, m_zone::getTurretShootingPose);
        },
        m_turret);
  }
}
