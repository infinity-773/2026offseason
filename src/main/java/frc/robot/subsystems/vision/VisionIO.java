// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /**
   * Represents a robot pose sample used for pose estimation.
   *
   * <p>The main vision subsystem can then decide to accept or reject it
   */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      double yaw,
      double pitch,
      double area,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    PHOTONVISION
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
