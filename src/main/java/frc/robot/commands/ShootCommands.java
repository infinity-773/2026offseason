package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class ShootCommands {

  public static Command shoot(Shooter shooter, Intake intake) {
    return Commands.startEnd(() -> shooter.shoot(0.4, 60), () -> shooter.stop(), shooter);
  }

  public static Command shootWithTime(Shooter shooter, Intake intake, double time) {
    return Commands.startEnd(
            () -> {
              shooter.shoot(0.4, 60);
              intake.setPos(() -> 0.27);
            },
            () -> {
              shooter.stop();
              intake.setPos(() -> 0.0);
            },
            shooter)
        .withTimeout(time);
  }
}
