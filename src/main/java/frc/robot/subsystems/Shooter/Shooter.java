package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public boolean isAtGoalSpeed = false;
  public boolean isAtGoalPos = true;
  public boolean readyFeed = false;
  public boolean startFeeder1 = false;
  private final Debouncer speedDebouncer = new Debouncer(0.2, DebounceType.kRising);
  private final Debouncer posDebouncer =
      new Debouncer(0.2, DebounceType.kRising); // this will fall when shoot out.use Both!
  private final Debouncer feedDebouncer =
      new Debouncer(
          0.2,
          DebounceType
              .kBoth); // avoid readyFeed falling when shoot cus bounce time is about 0.2~0.4;

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
    isAtGoalPos =
        posDebouncer.calculate(Math.abs(goalPos - inputs.shooterPosition) < 0.02); // 未设置齿轮比
    Logger.recordOutput("Shooter/atGoalPos", isAtGoalPos);
    // wait speed and pos to feed
    readyFeed =
        feedDebouncer.calculate(
            isAtGoalPos == true && isAtGoalSpeed == true && inputs.shotVelocitySetPoint != 0);

    if (readyFeed == true && startFeeder1 == true) {
      setFeeder_1Vol(ShooterConstants.FEEDER_1VOL);
    } else {
      setFeeder_1Vol(0);
    }
    Logger.recordOutput("Shooter/readFeed", readyFeed);
  }

  public void shoot(double Pos, double Feeder_Vel, double Shoot_Vel) {
    setPos(Pos); // 0.9 max
    setFeeder_2Velocity(Feeder_Vel);
    setShootVelocity(Shoot_Vel);
    startFeeder1 = true;
  }

  public void stop() {
    setPos(0);
    setFeeder_2Velocity(0);
    setShootVelocity(0);
    startFeeder1 = false;
  }

  private void setShootVelocity(double velocity) {
    io.setShooterVelocity(velocity);
  }

  private void setPos(double pos) {
    io.setShooterPos(pos);
  }

  private void setFeeder_1Vol(double vol) {
    io.setFeeder_1Vol(vol);
  }

  private void setFeeder_2Velocity(double velocity) {
    io.setFeeder_2Velocity(velocity);
  }
}
