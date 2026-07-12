package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class DriveToShootPoseCommand extends Command {
  // private final Shooter shooter;
  private final Drive drive;
  private Pose2d currentPose = new Pose2d();
  private Translation2d targetPoint = new Translation2d(); // 目标点坐标
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.1,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(Units.degreesToRadians(360.0), 5)); // TODO
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(0.1, 0.0, 0.0, new TrapezoidProfile.Constraints(5, 5));
  private double thetaErrorAbs = 0.0;
  private double driveErrorAbs = 0.0;
  private Translation2d lastSetpointTranslation = new Translation2d();

  private final double DISTANCE_1 = 2.3;

  public DriveToShootPoseCommand(Drive drive) {
    // this.shooter = shooter;
    this.drive = drive;
    // addRequirements(shooter);
    addRequirements(drive);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {

    // =====init target=====//
    // 原点永远定义在蓝方一侧

    if (ifBlue()) {
      targetPoint = new Translation2d(4.620, 4.045); // TODO
    } else {
      targetPoint = new Translation2d(11.8, 4.045); // TODO
    }

    currentPose = drive.getPose();

    ChassisSpeeds fieldVelocity =
        ChassisSpeeds.fromFieldRelativeSpeeds(drive.getChassisSpeeds(), currentPose.getRotation());
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);

    driveController.reset(
        currentPose.getTranslation().getDistance(targetPoint),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(targetPoint.minus(currentPose.getTranslation()).getAngle().unaryMinus())
                .getX()));

    thetaController.setTolerance(Units.degreesToRadians(1.0));
    driveController.setTolerance(0.03);

    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {

    currentPose = drive.getPose();

    // ===================================计算shooter目标速度==================================//
    double vel =
        Math.sqrt(
            currentPose.getTranslation().getDistance(targetPoint)
                * (ShooterConstants.G / Math.sin(2 * ShooterConstants.SHOOTER_POS)));
    if (vel > 70) vel = 70.0; // 限制最大射速

    // ======================================计算底盘转向速度=============================//
    Rotation2d goalRotation = getGoalRotation(currentPose.getTranslation()); // 燃料站与车连线方向 = 目标方向
    Rotation2d rotationError = goalRotation.minus(currentPose.getRotation()); // 目标方向-当前方向
    thetaErrorAbs = Math.abs(rotationError.getRadians()); // 弧度误差绝对值

    double thetaFFScaler =
        MathUtil.clamp((Units.radiansToDegrees(thetaErrorAbs) / Math.PI), 0.0, 1.0); // 计算动态前馈参数

    double thetaVelocity =
        thetaController.getSetpoint().velocity * thetaFFScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), goalRotation.getRadians());

    // ===========================================计算底盘平移速度========================//
    Translation2d driveGaolPoint = createPointOnLine(currentPose, targetPoint, DISTANCE_1);
    double currentDistance = currentPose.getTranslation().getDistance(driveGaolPoint);

    // use a small feedforward scaler if distance to target is small （0.10-0.15m？)
    double ffScaler = MathUtil.clamp((currentDistance) / (0.1), 0.0, 1.0);
    driveErrorAbs = currentDistance;

    // reset integral term which we don't need, the setpoint should not change because we pass in
    // the same value
    driveController.reset(
        lastSetpointTranslation.getDistance(driveGaolPoint),
        driveController.getSetpoint().velocity);

    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(
                driveErrorAbs,
                0.0); // this updates the current setpoint = the current distance to target

    // don't move if at goal
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;

    // goal is always 0,
    // setpoint is current distance to goal
    // lastSetpointTranslation = targetpose + difference from current pose to target
    lastSetpointTranslation =
        new Pose2d(
                driveGaolPoint,
                currentPose
                    .getTranslation()
                    .minus(driveGaolPoint)
                    .getAngle()) // angle from target to current pose
            .transformBy(
                new Transform2d(driveController.getSetpoint().position, 0.0, new Rotation2d()))
            .getTranslation();

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(), currentPose.getTranslation().minus(driveGaolPoint).getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();

    // ========================判断角度达到目标===========================//
    if (thetaErrorAbs < thetaController.getPositionTolerance()) {
      thetaVelocity = 0.0;
    }

    // =========================command drive========================//
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), -thetaVelocity, currentPose.getRotation()));

    // =======log states=======//

    Logger.recordOutput(
        "DriveToPoseCommand/fuelStationPose", new Pose2d(targetPoint, goalRotation));
    Logger.recordOutput("DriveToPoseCommand/GoalPose", new Pose2d(driveGaolPoint, goalRotation));
    Logger.recordOutput(
        "DriveToPoseCommand/currentSetPoint",
        new Pose2d(
            lastSetpointTranslation,
            Rotation2d.fromRadians(thetaController.getSetpoint().position)));
  }

  private boolean ifBlue() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  private Rotation2d getGoalRotation(Translation2d currentPoint) {
    Translation2d robotToTarget = targetPoint.minus(currentPoint);
    Rotation2d Goal = robotToTarget.getAngle();
    return Goal;
  }

  private Translation2d createPointOnLine(Pose2d robotPos, Translation2d targetPos, double radius) {
    // 计算从目标指向机器人的向量
    Translation2d direction = robotPos.getTranslation().minus(targetPos);

    // 计算当前距离
    double currentDist = direction.getNorm();

    // 计算单位向量
    Translation2d unitVector = direction.div(currentDist);

    // 计算目标点：B + unitVector * radius
    return targetPos.plus(unitVector.times(radius));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    // shooter.stop();
  }
}
