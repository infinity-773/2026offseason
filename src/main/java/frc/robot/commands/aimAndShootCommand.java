package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class aimAndShootCommand extends Command {
  private final Shooter shooter;
  private final Drive drive;
  private Pose2d currentPose = new Pose2d();
  private Translation2d targetPoint = new Translation2d(); // 目标点坐标
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.1,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(Units.degreesToRadians(360.0), 5)); // TODO

  private double thetaErrorAbs = 0.0;

  public aimAndShootCommand(Shooter shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;
    addRequirements(shooter);
    addRequirements(drive);

    // =====init target=====//
    // 原点永远定义在蓝方一侧
    if (ifBlue()) {
      targetPoint = new Translation2d(2, 2); // TODO
    } else {
      targetPoint = new Translation2d(4, 2); // TODO
    }

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    currentPose = drive.getPose();

    thetaController.reset(
        currentPose.getRotation().getRadians(),
        ChassisSpeeds.fromFieldRelativeSpeeds(drive.getChassisSpeeds(), currentPose.getRotation())
            .omegaRadiansPerSecond);

    thetaController.setTolerance(Units.degreesToRadians(1.0));
  }

  @Override
  public void execute() {

    currentPose = drive.getPose();

    // =============计算shooter目标速度===========//
    double vel =
        Math.sqrt(
            currentPose.getTranslation().getDistance(targetPoint)
                * (ShooterConstants.G / Math.sin(2 * ShooterConstants.SHOOTER_POS)));
    if (vel > 70) vel = 70.0; // 限制最大射速

    // ========计算底盘转向速度========//
    Rotation2d goalRotation = getGoalRotation(currentPose.getTranslation()); // 燃料站与车连线方向 = 目标方向
    Rotation2d rotationError = goalRotation.minus(currentPose.getRotation()); // 目标方向-当前方向
    thetaErrorAbs = Math.abs(rotationError.getRadians()); // 弧度误差绝对值

    double thetaFFScaler =
        MathUtil.clamp((Units.radiansToDegrees(thetaErrorAbs) / Math.PI), 0.0, 1.0); // 计算动态前馈参数

    double thetaVelocity =
        thetaController.getSetpoint().velocity * thetaFFScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), goalRotation.getRadians());

    // ======判断达到目标并shoot=========//
    if (thetaErrorAbs < thetaController.getPositionTolerance()) {
      thetaVelocity = 0.0;
      shooter.shoot(ShooterConstants.SHOOTER_POS, vel);
    } else {
      shooter.stop();
    }

    // ======command drive=======//
    drive.runVelocity(new ChassisSpeeds(0, 0, thetaVelocity));

    // =======log states=======//
    Logger.recordOutput("aimAndShootCommad/ThetaMeasured", currentPose.getRotation());
    Logger.recordOutput(
        "aimAndShootCommad/ThetaSetpoint",
        Rotation2d.fromRadians(thetaController.getSetpoint().position));
    Logger.recordOutput("aimAndShootCommad/fuelStationPose", new Pose2d(targetPoint, goalRotation));
    Logger.recordOutput("aimAndShootCommad/goalRotation", goalRotation);
    Logger.recordOutput("aimAndShootCommad/shooterVelocity", vel);
  }

  private boolean ifBlue() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  private Rotation2d getGoalRotation(Translation2d currentPoint) {
    Translation2d robotToTarget = targetPoint.minus(currentPoint);
    Rotation2d Goal = robotToTarget.getAngle();
    return Goal;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    shooter.stop();
  }
}
