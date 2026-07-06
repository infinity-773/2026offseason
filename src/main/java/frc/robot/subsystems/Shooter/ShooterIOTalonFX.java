package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shotMotor;
  private final TalonFX feedMotor_2;
  private final TalonFX feedMotor_1;
  private final TalonFX turnMotor;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Motion Magic requests
  private final MotionMagicVelocityVoltage speedMMrequest = new MotionMagicVelocityVoltage(0.0);
  private final MotionMagicVoltage turnMMreuqest = new MotionMagicVoltage(0.0);

  public ShooterIOTalonFX() {
    TalonFXConfiguration shotMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration feedMotorConfig_1 = new TalonFXConfiguration();
    TalonFXConfiguration feedMotorConfig_2 = new TalonFXConfiguration();
    TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
    //
    shotMotorConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.SHOT_MM_ACCELERATION;
    shotMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.SHOT_MM_CRUISE_VELOCITY;
    shotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    shotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    shotMotorConfig.Voltage.PeakForwardVoltage = ShooterConstants.PEAK_FORWARD_VOLTAGE;
    shotMotorConfig.Voltage.PeakReverseVoltage = ShooterConstants.PEAK_REVERSE_VOLTAGE;
    shotMotorConfig.Slot0.kS = ShooterConstants.SHOT_KS;
    shotMotorConfig.Slot0.kV = ShooterConstants.SHOT_KV;
    shotMotorConfig.Slot0.kA = ShooterConstants.SHOT_KA;
    shotMotorConfig.Slot0.kP = ShooterConstants.SHOT_KP;
    shotMotorConfig.Slot0.kD = ShooterConstants.SHOT_KD;
    shotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //
    turnMotorConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.TURN_MM_ACCELERATION;
    turnMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.TURN_MM_CRUISE_VELOCITY;
    turnMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    turnMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ShooterConstants.TURN_FORWARD_SOFT_LIMIT_THRESHOLD; // the max pos is about 2.47
    turnMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    turnMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ShooterConstants.TURN_REVERSE_SOFT_LIMIT_THRESHOLD;
    turnMotorConfig.Voltage.PeakForwardVoltage = ShooterConstants.PEAK_FORWARD_VOLTAGE;
    turnMotorConfig.Voltage.PeakReverseVoltage = ShooterConstants.PEAK_REVERSE_VOLTAGE;
    turnMotorConfig.Slot0.kS = ShooterConstants.TURN_KS;
    turnMotorConfig.Slot0.kV = ShooterConstants.TURN_KV;
    turnMotorConfig.Slot0.kG = ShooterConstants.TURN_KG;
    turnMotorConfig.Slot0.kA = ShooterConstants.TURN_KA;
    turnMotorConfig.Slot0.kP = ShooterConstants.TURN_KP;
    turnMotorConfig.Slot0.kD = ShooterConstants.TURN_KD;
    turnMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //
    feedMotorConfig_1.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    feedMotorConfig_1.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    feedMotorConfig_1.Voltage.PeakForwardVoltage = ShooterConstants.PEAK_FORWARD_VOLTAGE;
    feedMotorConfig_1.Voltage.PeakReverseVoltage = ShooterConstants.PEAK_REVERSE_VOLTAGE;
    feedMotorConfig_1.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //
    feedMotorConfig_2.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    feedMotorConfig_2.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    feedMotorConfig_2.Voltage.PeakForwardVoltage = ShooterConstants.PEAK_FORWARD_VOLTAGE;
    feedMotorConfig_2.Voltage.PeakReverseVoltage = ShooterConstants.PEAK_REVERSE_VOLTAGE;
    feedMotorConfig_2.MotionMagic.MotionMagicAcceleration = ShooterConstants.FEED_2_MM_ACCELERATION;
    feedMotorConfig_2.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.FEED_2_MM_CRUISE_VELOCITY;
    feedMotorConfig_2.Slot0.kS = ShooterConstants.FEED_2_KS;
    feedMotorConfig_2.Slot0.kV = ShooterConstants.FEED_2_KV;
    feedMotorConfig_2.Slot0.kP = ShooterConstants.FEED_2_KP; // 不要改到0.6以上
    feedMotorConfig_2.Slot0.kD = ShooterConstants.FEED_2_KD;
    feedMotorConfig_2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    this.shotMotor = new TalonFX(ShooterConstants.SHOT_MOTOR_ID);
    this.feedMotor_2 = new TalonFX(ShooterConstants.FEED_MOTOR_2_ID); // shooter上的滚筒
    this.feedMotor_1 = new TalonFX(ShooterConstants.FEED_MOTOR_1_ID); // feeder_1 底盘上的多组皮带
    this.turnMotor = new TalonFX(ShooterConstants.TURN_MOTOR_ID);

    turnMotor.getConfigurator().apply(turnMotorConfig);
    shotMotor.getConfigurator().apply(shotMotorConfig);
    feedMotor_1.getConfigurator().apply(feedMotorConfig_1);
    feedMotor_2.getConfigurator().apply(feedMotorConfig_2);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterVelocity = shotMotor.getVelocity().getValueAsDouble();
    inputs.shooterCurrentAmps = shotMotor.getSupplyCurrent().getValueAsDouble();
    inputs.feederVelocity = feedMotor_2.getVelocity().getValueAsDouble();
    inputs.shooterPosition = turnMotor.getPosition().getValueAsDouble();
    inputs.shotVelocitySetPoint = shotMotor.getClosedLoopReference().getValueAsDouble();
    inputs.feedVelSetpoint = feedMotor_2.getClosedLoopReference().getValueAsDouble();
    inputs.positionSetPoint = turnMotor.getClosedLoopReference().getValueAsDouble();
  }

  @Override
  public void setShooterVelocity(double velocity) {
    shotMotor.setControl(speedMMrequest.withVelocity(velocity));
  }

  @Override
  public void setShooterPos(double pos) {
    turnMotor.setControl(turnMMreuqest.withPosition(pos));
  }

  @Override
  public void setFeeder_1Vol(double vol) {
    feedMotor_1.setControl(voltageRequest.withOutput(vol));
  }

  @Override
  public void setFeeder_2Velocity(double velocity) {
    feedMotor_2.setControl(speedMMrequest.withVelocity(velocity));
  }

  @Override
  public void zeroPos(double pos) {
    turnMotor.setPosition(0.0);
  }
}
