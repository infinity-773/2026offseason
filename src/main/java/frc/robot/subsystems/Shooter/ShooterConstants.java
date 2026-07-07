package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Constants for the Shooter subsystem, including CAN IDs, PID/FF gains, Motion Magic parameters,
 * and soft limit thresholds.
 */
public final class ShooterConstants {
  // Shooter
  public static final double FEEDER_1VOL = 6;
  public static final double SHOOTER_POS = 0;
  public static final Translation2d GOAL = new Translation2d(0, 0);
  public static final double G = 9.8;

  // ShooterIOTalonFX
  // ========== CAN IDs ==========
  public static final int SHOT_MOTOR_ID = 15;
  public static final int FEED_MOTOR_2_ID = 16; // shooter
  public static final int FEED_MOTOR_1_ID = 17; // feeder
  public static final int TURN_MOTOR_ID = 18;

  // ========== Shot Motor ==========
  public static final double SHOT_MM_ACCELERATION = 60.0;
  public static final double SHOT_MM_CRUISE_VELOCITY = 60.0;
  public static final double SHOT_KS = 0.22;
  public static final double SHOT_KV = 0.115;
  public static final double SHOT_KA = 0.0;
  public static final double SHOT_KP = 0.2;
  public static final double SHOT_KD = 0.0;

  // ========== Turn Motor  ==========
  public static final double TURN_MM_ACCELERATION = 25.0;
  public static final double TURN_MM_CRUISE_VELOCITY = 20.0;
  public static final double TURN_FORWARD_SOFT_LIMIT_THRESHOLD = 2.1; // max pos ≈ 2.47
  public static final double TURN_REVERSE_SOFT_LIMIT_THRESHOLD = -0.3;
  public static final double TURN_KS = 0.1;
  public static final double TURN_KV = 0.1;
  public static final double TURN_KG = 0.225;
  public static final double TURN_KA = 0.0;
  public static final double TURN_KP = 1.5;
  public static final double TURN_KD = 0.2;

  // ========== Feed Motor 2  ==========
  public static final double FEED_2_MM_ACCELERATION = 45.0;
  public static final double FEED_2_MM_CRUISE_VELOCITY = 30.0;
  public static final double FEED_2_KS = 0.22;
  public static final double FEED_2_KV = 0.116;
  public static final double FEED_2_KP = 0.5; // 不要改到0.6以上
  public static final double FEED_2_KD = 0.0;

  // ========== Voltage Limits ==========
  public static final double PEAK_FORWARD_VOLTAGE = 12.0;
  public static final double PEAK_REVERSE_VOLTAGE = -12.0;
}
