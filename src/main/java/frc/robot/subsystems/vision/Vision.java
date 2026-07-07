// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib6907.DualEdgeDelayedBoolean;
import frc.robot.lib6907.DualEdgeDelayedBoolean.EdgeType;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  // private final Supplier<Translation2d> velocitySupplier;

  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // keep track of multi-tag usage with a delayed boolean for each camera
  private final DualEdgeDelayedBoolean[] multiTagDelayedBooleans;
  private final Translation2d[] lastAcceptedPose2d; // For velocity gating
  private final double[] lastAcceptedTimestamp;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    // this.velocitySupplier = velocitySupplier;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    // delayed boolean and last accepted pose for each camera
    multiTagDelayedBooleans = new DualEdgeDelayedBoolean[io.length];
    lastAcceptedPose2d = new Translation2d[io.length];
    lastAcceptedTimestamp = new double[io.length];
    for (int i = 0; i < io.length; i++) {
      multiTagDelayedBooleans[i] = new DualEdgeDelayedBoolean(MULTI_TAG_DELAY, EdgeType.RISING);
      lastAcceptedPose2d[i] = null;
      lastAcceptedTimestamp[i] = 0.0;
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
      if (!inputs[cameraIndex].connected) {
        continue;
      }

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // multi tag usage check
      boolean sawMultiTag = false;
      for (PoseObservation obs : inputs[cameraIndex].poseObservations) {
        if (obs.tagCount() > 1) {
          sawMultiTag = true;
          break;
        }
      }
      boolean multiTagActive = multiTagDelayedBooleans[cameraIndex].update(sawMultiTag);

      // Loop over pose observations
      for (PoseObservation obs : inputs[cameraIndex].poseObservations) {
        double timestamp = obs.timestamp();
        Pose3d rawPose = obs.pose();
        robotPoses.add(rawPose);

        // -------- 1) Basic checks like tagCount > 0, ambiguity, Z margin ------
        if (obs.tagCount() != 1 && multiTagActive == false) {
          robotPosesRejected.add(rawPose);
          continue;
        }
        if (obs.ambiguity() > ACCEPTABLE_AMBIGUITY_THRESHOLD) {
          robotPosesRejected.add(rawPose);
          continue;
        }
        if (Math.abs(rawPose.getZ()) > Z_MARGIN) {
          robotPosesRejected.add(rawPose);
          continue;
        }
        if (obs.area() < MIN_AREA[cameraIndex]) {
          robotPosesRejected.add(rawPose);
          continue;
        }

        // yaw pitch filter
        boolean yawPitchReject =
            (Math.abs(obs.yaw()) > ACCEPTABLE_YAW_THRESHOLD)
                || (Math.abs(obs.pitch()) > ACCEPTABLE_PITCH_THRESHOLD);
        if (yawPitchReject) {
          robotPosesRejected.add(rawPose);
          continue;
        }

        // -------- 2) Velocity gating -----------
        // Translation2d chassisVelocity = velocitySupplier.get();

        // if (lastAcceptedPose2d[cameraIndex] != null
        // && Math.abs(timestamp - lastAcceptedTimestamp[cameraIndex]) > 0.0001) {
        // double dt = timestamp - lastAcceptedTimestamp[cameraIndex];
        // Translation2d newT2d = rawPose.getTranslation().toTranslation2d();
        // Translation2d oldT2d = lastAcceptedPose2d[cameraIndex];
        // Translation2d visionVel = newT2d.minus(oldT2d).div(dt);

        // double deltav = visionVel.minus(chassisVelocity).getNorm();
        // if ((chassisVelocity.getNorm() < 0.5 && deltav > STATIC_DELTAV_BOUND)
        // || (chassisVelocity.getNorm() >= 0.5 && deltav > MOVING_DELTAV_BOUND)) {
        // // Reject
        // robotPosesRejected.add(rawPose);
        // continue;
        // }
        // }
        // Passed all filters => ACCEPT
        robotPosesAccepted.add(rawPose);
        lastAcceptedPose2d[cameraIndex] = rawPose.getTranslation().toTranslation2d();
        lastAcceptedTimestamp[cameraIndex] = timestamp;

        // ==== 3) Compute standard deviations ====
        double distFactor = Math.pow(obs.averageTagDistance(), 2.0) / Math.max(obs.tagCount(), 1);
        double linearStdDev = linearStdDevBaseline * distFactor;
        double angularStdDev = angularStdDevBaseline * distFactor;

        // If multi-tag is active and we actually used multiple tags => better angle
        if (!(multiTagActive
            && obs.tagCount() > 1
            && obs.type() == PoseObservationType.PHOTONVISION)) {
          angularStdDev *= 2.0;
        }

        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // ======= 4) Send the final observation to your pose consumer =======
        Matrix<N3, N1> measurementStdDevs =
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
        consumer.accept(rawPose.toPose2d(), timestamp, measurementStdDevs);
      }
      // =========== Log results for this camera ===========
      recordLogs(cameraIndex, tagPoses, robotPoses, robotPosesAccepted, robotPosesRejected);
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  private void recordLogs(
      int cameraIndex,
      List<Pose3d> tagPoses,
      List<Pose3d> cameraPoses,
      List<Pose3d> cameraPosesAccepted,
      List<Pose3d> cameraPosesRejected) {

    Logger.recordOutput(
        "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Camera" + cameraIndex + "/RobotPoses", cameraPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
        cameraPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
        cameraPosesRejected.toArray(new Pose3d[0]));
  }
}
