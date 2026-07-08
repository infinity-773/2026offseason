package frc.robot.subsystems.Intake;

/**
 * Constants for the Intake subsystem, including CAN IDs, PID/FF gains, Motion Magic parameters, and
 * gear ratio configurations.
 */
public final class IntakeConstants {
  private IntakeConstants() {} // Utility class, prevent instantiation

  // ========== CAN IDs ==========
  public static final int INTAKE_MOTOR_ID = 13;
  public static final int TURN_MOTOR_ID = 14;

  // ========== Turn Motor ==========
  // Gravity feedforward (Arm_Cosine mode)
  public static final double TURN_KG = 1.23;
  public static final double TURN_KS = 0.2;
  public static final double TURN_KV = 0.3;
  public static final double TURN_KA = 0.0;
  public static final double TURN_KP = 8.8;
  public static final double TURN_KI = 0.0;
  public static final double TURN_KD = 1.3;

  // Motion Magic
  public static final double TURN_MM_CRUISE_VELOCITY = 5.0;
  public static final double TURN_MM_ACCELERATION = 5.0;

  // Gear ratio
  public static final double TURN_ROTOR_TO_SENSOR = 1.0; // 电机转子1转，传感器1转
  public static final double TURN_SENSOR_TO_MECHANISM = 5.555; // 电机转子50/9转，arm 1转

  // ========== Intake Motor ==========
  public static final double INTAKE_KS = 0.0;
  public static final double INTAKE_KV = 0.0;
  public static final double INTAKE_KP = 0.0;
  public static final double INTAKE_KI = 0.0;
  public static final double INTAKE_KD = 0.0;

  // ========== Voltage Limits ==========
  public static final double PEAK_FORWARD_VOLTAGE = 12.0;
  public static final double PEAK_REVERSE_VOLTAGE = -12.0;
}
