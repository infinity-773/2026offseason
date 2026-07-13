package frc.robot.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class AutoFactory {

  public static Command buildPathCommand(String fileName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(fileName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public static PathPlannerPath getPPPath(String fileName) {
    try {
      return PathPlannerPath.fromPathFile(fileName);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }

  public static void setPoseTostartPoint(PathPlannerPath firstPath, Drive drive) {
    drive.setPose(firstPath.getStartingHolonomicPose().get());
  }

  // public static
}
