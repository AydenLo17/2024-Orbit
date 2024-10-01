// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class IntakeIOKrakenFOC implements IntakeIO {

  private final TalonFX intakeTalon;

  private final StatusSignal<Double> intakePosition;
  private final StatusSignal<Double> intakeVelocity;
  private final StatusSignal<Double> intakeAppliedVolts;
  private final StatusSignal<Double> intakeSupplyCurrent;
  private final StatusSignal<Double> intakeTorqueCurrent;
  private final StatusSignal<Double> intakeTempCelsius;

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  public IntakeIOKrakenFOC() {
    intakeTalon = new TalonFX(15, "rio");

    // General config
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Apply configs
    intakeTalon.getConfigurator().apply(config, 1.0);

    // Set inverts
    intakeTalon.setInverted(true);

    // Set signals
    intakePosition = intakeTalon.getPosition();
    intakeVelocity = intakeTalon.getVelocity();
    intakeAppliedVolts = intakeTalon.getMotorVoltage();
    intakeSupplyCurrent = intakeTalon.getSupplyCurrent();
    intakeTorqueCurrent = intakeTalon.getTorqueCurrent();
    intakeTempCelsius = intakeTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        intakePosition,
        intakeVelocity,
        intakeAppliedVolts,
        intakeSupplyCurrent,
        intakeTorqueCurrent,
        intakeTempCelsius);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorConnected = BaseStatusSignal.refreshAll(
        intakePosition,
        intakeVelocity,
        intakeAppliedVolts,
        intakeSupplyCurrent,
        intakeTorqueCurrent,
        intakeTempCelsius)
        .isOK();

    inputs.intakePositionRads = Units.rotationsToRadians(intakePosition.getValueAsDouble());
    inputs.intakeVelocityRpm = intakeVelocity.getValueAsDouble() * 60.0;
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeSupplyCurrentAmps = intakeSupplyCurrent.getValueAsDouble();
    inputs.intakeTorqueCurrentAmps = intakeTorqueCurrent.getValueAsDouble();
    inputs.intakeTempCelsius = intakeTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double intakeVolts) {
    intakeTalon.setControl(voltageControl.withOutput(intakeVolts));
  }

  @Override
  public void stop() {
    intakeTalon.setControl(neutralControl);
  }

  @Override
  public void runCharacterizationIntake(double input) {
    intakeTalon.setControl(voltageControl.withOutput(input));
  }
}
