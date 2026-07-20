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

public class Middle_A extends Command {

  public static Command runBuleA1CommandInAuto(
      String AutoName1, Intake intake, Shooter shooter) {

    return new SequentialCommandGroup(
        new PathPlannerAuto(AutoName1),
        ShootCommands.shootWithTime(shooter, intake, 4));
  }
}
