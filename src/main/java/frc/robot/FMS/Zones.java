// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FMS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.WaypointConstants;
import frc.robot.constants.ZonesConstants.Zone;

import java.util.Optional;
import java.util.function.Supplier;

/** Add your docs here. */
public class Zones {
  private static Supplier<Pose2d> m_pose;

  private static Zone CurrentZone = Zone.ALLIANCE_ZONE;
  private Optional<Alliance> m_team;

  public Zones(Supplier<Pose2d> robotPoseSupplier) {
    m_team = DriverStation.getAlliance();
    m_pose = robotPoseSupplier;
  }

  public void updateZone() {
    double x = m_pose.get().getX();
    double y = m_pose.get().getY();

    SmartDashboard.putString("CurrentZone", CurrentZone.name());

    if (isWithin(x, y, Zone.ALLIANCE_ZONE.getTopLeftTranslation(), Zone.ALLIANCE_ZONE.getBottomRightTranslation())) {
      CurrentZone = Zone.ALLIANCE_ZONE;
    } else if (isWithin(x, y, Zone.NEUTRAL_ZONE.getTopLeftTranslation(), Zone.NEUTRAL_ZONE.getBottomRightTranslation())) {
      CurrentZone = Zone.NEUTRAL_ZONE;
    } else if (isWithin(x, y, Zone.OPPONENT_ZONE.getTopLeftTranslation(), Zone.OPPONENT_ZONE.getBottomRightTranslation())) {
      CurrentZone = Zone.OPPONENT_ZONE;
    } else {
      // Outside defined zones, handle as needed
      CurrentZone = Zone.OUT_OF_BOUNDS;
    }
  }

  public Pose2d getTurretShootingPose() {
    if (!m_team.isPresent()) {
      return WaypointConstants.blueHub;
    }
    switch (CurrentZone) {
      case ALLIANCE_ZONE:
        return m_team.get() == Alliance.Blue ? WaypointConstants.blueHub : WaypointConstants.redHub;
      case NEUTRAL_ZONE:
        return m_team.get() == Alliance.Blue ? WaypointConstants.blueHub : WaypointConstants.redHub;
      case OPPONENT_ZONE:
        return m_team.get() == Alliance.Blue ? WaypointConstants.blueHub : WaypointConstants.redHub;
      default:
        break;
    }
    return m_team.get() == Alliance.Blue ? WaypointConstants.blueHub : WaypointConstants.redHub;
  }

  private boolean isWithin(double x, double y, Translation2d corner1, Translation2d corner2) {
    double minX = Math.min(corner1.getX(), corner2.getX());
    double maxX = Math.max(corner1.getX(), corner2.getX());
    double minY = Math.min(corner1.getY(), corner2.getY());
    double maxY = Math.max(corner1.getY(), corner2.getY());

    return (x >= minX && x <= maxX) && (y >= minY && y <= maxY);
  }

  public Zone getZone() {
    return CurrentZone;
  }
}
