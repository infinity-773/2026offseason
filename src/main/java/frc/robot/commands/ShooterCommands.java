package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class ShooterCommands extends Command {
  private final Shooter shooter;
  private final Translation2d translation2d;

  public ShooterCommands(Shooter shooter, Translation2d translation2d) {
    this.shooter = shooter;
    this.translation2d = translation2d;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    double vel = Math.sqrt(translation2d.getDistance(ShooterConstants.GOAL)*(ShooterConstants.G/Math.sin(2 * ShooterConstants.SHOOTER_POS)));
    shooter.shoot(ShooterConstants.SHOOTER_POS, ShooterConstants.FEEDER_1VOL, vel);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
