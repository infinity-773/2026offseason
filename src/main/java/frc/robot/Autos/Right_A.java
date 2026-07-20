package frc.robot.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.intakeCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drive.Drive;

public class Right_A extends Command {

  public static Command BlueA(Intake intake, Shooter shooter, Drive drive) {
    return new SequentialCommandGroup(
        runBuleA1Command("BlueRight_A1", "BlueRight_A2", intake, shooter, drive),
        runBuleA2Command("BlueRight_A3", "BlueRight_A2", intake, shooter));
  }

  public static Command runBuleA1Command(
      String pathName1, String pathName2, Intake intake, Shooter shooter, Drive drive) {

    PathPlannerPath path1 = AutoFactory.getPPPath(pathName1);
    PathPlannerPath path2 = AutoFactory.getPPPath(pathName2);
    AutoFactory.setPoseTostartPoint(path1, drive);

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(AutoBuilder.followPath(path1), new intakeCommand(intake)),
        AutoBuilder.followPath(path2),
        ShootCommands.shootWithTime(shooter, intake, 5));
  }

  public static Command runBuleA2Command(
      String pathName1, String pathName2, Intake intake, Shooter shooter) {
    PathPlannerPath path1 = AutoFactory.getPPPath(pathName1);
    PathPlannerPath path2 = AutoFactory.getPPPath(pathName2);

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(AutoBuilder.followPath(path1), new intakeCommand(intake)),
        AutoBuilder.followPath(path2),
        ShootCommands.shootWithTime(shooter, intake, 5));
  }

  public static Command runBuleA1CommandInAuto(
      String AutoName1, String AutoName2, String AutoName3, Intake intake, Shooter shooter) {

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(new PathPlannerAuto(AutoName1), new intakeCommand(intake)),
        new PathPlannerAuto(AutoName2),
        ShootCommands.shootWithTime(shooter, intake, 5),
        new PathPlannerAuto(AutoName3),
        new PathPlannerAuto(AutoName2),
        ShootCommands.shootWithTime(shooter, intake, 5));
  }
}
