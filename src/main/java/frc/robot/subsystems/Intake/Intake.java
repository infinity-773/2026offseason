package frc.robot.subsystems.Intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private boolean isAtGoal = false;
  private final Debouncer atGoalDebouncer = new Debouncer(0.1, DebounceType.kRising);
  private final DoubleSupplier GoalPos = () -> 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
    io.resetPos(0);

    // setIntakeRest();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    isAtGoal =
        atGoalDebouncer.calculate(Math.abs(inputs.turnPosition - inputs.positionSetPoint) < 0.02);
    Logger.recordOutput("Intake/atGoal", isAtGoal);
    if (isAtGoal == true
        && inputs.positionSetPoint == 0.0) { // do not run profiled PID when at intake
      io.hold(-2); // TODO
    } else {
      setPos(GoalPos.getAsDouble());
    }
  }

  private void setPos(double position) { // intake的位置大约在+2.0；
    io.setPosition(position);
  }

  public void intake(double vol) {
    io.setVol(vol);
  }

  public boolean intakeAtGoal() {
    return this.isAtGoal;
  }

  public void setIntakeRest() {
    io.setPosition(1.0);
    io.setVol(0.0);
  }

  public void zero() {}
}
