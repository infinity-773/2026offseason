package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.drive.Drive;

public class ShooterCommands extends Command {
  private final Shooter shooter;
  private final Drive drive;
  private Translation2d translation2d;

  public ShooterCommands(Shooter shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    translation2d = drive.getPose().getTranslation();
    double vel =
        Math.sqrt(
            translation2d.getDistance(ShooterConstants.GOAL)
                * (ShooterConstants.G / Math.sin(2 * ShooterConstants.SHOOTER_POS)));
    shooter.shoot(ShooterConstants.SHOOTER_POS, vel);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
