package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.WaypointConstants;

public class CommandFactory {

    private CommandSwerveDrivetrain m_drivetrain;
    private int m_side = 1;
    private boolean m_running = false;

    public CommandFactory(
      CommandSwerveDrivetrain drivetrain
      ) {
        this.m_drivetrain = drivetrain;
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
        Set.of(m_drivetrain)
    ).repeatedly();
}

}