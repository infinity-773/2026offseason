package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeVelocity = 0.0;
    public double turnAmps = 0.0;
    public double intakeAmps = 0.0;
    public double turnPosition = 0.0;
    public double positionSetPoint = 0.0;
    public double velocitySetPoint = 0.0;
  }

  public default void setPosition(double position) {}

  public default void setVol(double vol) {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void resetPos(double position) {}

  public default void hold(double vol) {}
}
