package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private boolean isAtGoalSpeed = false;
  private boolean isAtGoalPos = true;
  private final Debouncer speedDebouncer = new Debouncer(0.2, DebounceType.kRising);
  private final Debouncer posDebouncer = new Debouncer(0.2, DebounceType.kRising);
  private DoubleSupplier goalSpeedSupplier = () -> 0.0;
  private DoubleSupplier goalPosSupplier = () -> 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
    io.zeroPos(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("shooter", inputs);
    // 更新飞轮是否达到设定速度
    double goalSpeed = inputs.shotVelocitySetPoint;
    isAtGoalSpeed = speedDebouncer.calculate(Math.abs(goalSpeed - inputs.shooterVelocity) < 2);
    Logger.recordOutput("Shooter/atGoalSpeed", isAtGoalSpeed);
    // 更新是否达到设定位置
    double goalPos = inputs.positionSetPoint;
    isAtGoalPos = posDebouncer.calculate(Math.abs(goalPos - inputs.shooterPosition) < 0.3);
    Logger.recordOutput("Shooter/atGoalPos", isAtGoalPos);
  }

  public void shootWithPos(double pos) {}

  public void setShootVelocity(double velocity) {
    io.setShooterVelocity(velocity);
  }

  public void setPos(double pos) {
    io.setShooterPos(pos);
  }

  public void setFeeder_1Vol(double vol) {
    io.setFeeder_1Vol(vol);
  }

  public void setFeeder_2Velocity(double velocity) {
    io.setFeeder_2Velocity(velocity);
  }
}
