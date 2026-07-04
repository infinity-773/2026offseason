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
  private final Debouncer atGoalDebouncer = new Debouncer(0.2, DebounceType.kRising);
  private final Debouncer shouldHoldDebouncer = new Debouncer(0.2, DebounceType.kRising);
  private DoubleSupplier goalSupplier = () -> 0.0;

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
        atGoalDebouncer.calculate(
            Math.abs(inputs.turnPosition - goalSupplier.getAsDouble()) < 0.08);
    Logger.recordOutput("Intake/atGoal", isAtGoal);
    if (isAtGoal) {
      if (Math.abs(inputs.turnPosition) < 0.07) {
        io.hold(-0.2);
      }
    } else {
      io.setPosition(goalSupplier.getAsDouble()); // TODO
    }
  }

  public void setPos(DoubleSupplier position) {
    this.goalSupplier = position;
  }

  public void intake(double vol) {
    io.setVol(vol);
  }

  public boolean intakeAtGoal() {
    return this.isAtGoal;
  }
}
