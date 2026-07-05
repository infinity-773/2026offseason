package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX turnMotor;
  private final TalonFX intakeMotor;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Motion magic
  private final MotionMagicVoltage armMMcontrol = new MotionMagicVoltage(0.0);
  private final MotionMagicVelocityVoltage velocityMMcontrol = new MotionMagicVelocityVoltage(0.0);

  public IntakeIOTalonFX() {
    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    turnConfig.Slot0.kG = IntakeConstants.TURN_KG;
    turnConfig.Slot0.kS = IntakeConstants.TURN_KS;
    turnConfig.Slot0.kA = IntakeConstants.TURN_KA;
    turnConfig.Slot0.kV = IntakeConstants.TURN_KV;
    turnConfig.Slot0.kP = IntakeConstants.TURN_KP;
    turnConfig.Slot0.kI = IntakeConstants.TURN_KI;
    turnConfig.Slot0.kD = IntakeConstants.TURN_KD;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.TURN_MM_CRUISE_VELOCITY;
    turnConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.TURN_MM_ACCELERATION;
    turnConfig.Voltage.PeakForwardVoltage = IntakeConstants.PEAK_FORWARD_VOLTAGE;
    turnConfig.Voltage.PeakReverseVoltage = IntakeConstants.PEAK_REVERSE_VOLTAGE;
    turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    turnConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    turnConfig.Feedback.RotorToSensorRatio = IntakeConstants.TURN_ROTOR_TO_SENSOR; // 电机转子1转，传感器1转。
    turnConfig.Feedback.SensorToMechanismRatio =
        IntakeConstants.TURN_SENSOR_TO_MECHANISM; // 电机转子50/9转，arm1转。
    turnConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    turnConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    turnConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
    turnConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
    intakeConfig.Slot0.kV = IntakeConstants.INTAKE_KV;
    intakeConfig.Slot0.kS = IntakeConstants.INTAKE_KS;
    intakeConfig.Slot0.kP = IntakeConstants.INTAKE_KP;
    intakeConfig.Slot0.kI = IntakeConstants.INTAKE_KI;
    intakeConfig.Slot0.kD = IntakeConstants.INTAKE_KD;
    intakeConfig.Voltage.PeakForwardVoltage = IntakeConstants.PEAK_FORWARD_VOLTAGE;
    intakeConfig.Voltage.PeakReverseVoltage = IntakeConstants.PEAK_REVERSE_VOLTAGE;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    intakeConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    this.turnMotor = new TalonFX(IntakeConstants.TURN_MOTOR_ID);
    this.intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    turnMotor.getConfigurator().apply(turnConfig);
    intakeMotor.getConfigurator().apply(intakeConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.turnPosition = turnMotor.getPosition().getValueAsDouble();
    inputs.intakeVelocity = intakeMotor.getVelocity().getValueAsDouble();
    inputs.turnAmps = turnMotor.getSupplyCurrent().getValueAsDouble();
    inputs.intakeAmps = intakeMotor.getSupplyCurrent().getValueAsDouble();
    inputs.positionSetPoint = turnMotor.getClosedLoopReference().getValueAsDouble();
    inputs.velocitySetPoint = intakeMotor.getClosedLoopReference().getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {
    turnMotor.setControl(armMMcontrol.withPosition(position));
  }

  @Override
  public void setVol(double vol) {
    intakeMotor.setControl(voltageRequest.withOutput(vol));
  }

  @Override
  public void resetPos(double position) {
    turnMotor.setPosition(position);
  }

  @Override
  public void hold(double vol) {
    turnMotor.setControl(voltageRequest.withOutput(vol));
  }
}
