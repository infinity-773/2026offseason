package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterVelocity = 0.0;
    public double shooterCurrentAmps = 0.0;
    public double feederVelocity = 0.0;
    public double feederCurrentAmps = 0.0;
    public double shooterPosition = 0.0;
    public double shotVelocitySetPoint = 0.0;
    public double feedVelSetpoint = 0.0;
    public double positionSetPoint = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterVelocity(double velocity) {}

  public default void setFeeder_1Vol(double vol) {}

  public default void setFeeder_2Velocity(double velocity) {}

  public default void setShooterPos(double position) {}

  public default void zeroPos(double pos) {}
}
