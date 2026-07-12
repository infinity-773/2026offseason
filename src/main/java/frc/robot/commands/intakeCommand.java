package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class intakeCommand extends Command {
  private final Intake intake;

  public intakeCommand(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    intake.intake(6);
  }

  @Override
  public void end(boolean interrupted) {
    intake.intake(0.0);
  }
}
