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

package frc.robot.subsystems.magazine;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.util.subsystem.AdvancedSubsystem;

public class MagazineIOKrakenFOC implements MagazineIO {

  private final TalonFX magazineTalon;

  private final StatusSignal<Double> magazinePosition;
  private final StatusSignal<Double> magazineVelocity;
  private final StatusSignal<Double> magazineAppliedVolts;
  private final StatusSignal<Double> magazineSupplyCurrent;
  private final StatusSignal<Double> magazineTorqueCurrent;
  private final StatusSignal<Double> magazineTempCelsius;

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  public MagazineIOKrakenFOC() {
    magazineTalon = new TalonFX(15, "rio");

    // General config
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Apply configs
    magazineTalon.getConfigurator().apply(config, 1.0);

    // Set inverts
    magazineTalon.setInverted(true);

    // Set signals
    magazinePosition = magazineTalon.getPosition();
    magazineVelocity = magazineTalon.getVelocity();
    magazineAppliedVolts = magazineTalon.getMotorVoltage();
    magazineSupplyCurrent = magazineTalon.getSupplyCurrent();
    magazineTorqueCurrent = magazineTalon.getTorqueCurrent();
    magazineTempCelsius = magazineTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        magazinePosition,
        magazineVelocity,
        magazineAppliedVolts,
        magazineSupplyCurrent,
        magazineTorqueCurrent,
        magazineTempCelsius);
  }

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    inputs.magazineMotorConnected =
        BaseStatusSignal.refreshAll(
                magazinePosition,
                magazineVelocity,
                magazineAppliedVolts,
                magazineSupplyCurrent,
                magazineTorqueCurrent,
                magazineTempCelsius)
            .isOK();

    inputs.magazinePositionRads = Units.rotationsToRadians(magazinePosition.getValueAsDouble());
    inputs.magazineVelocityRpm = magazineVelocity.getValueAsDouble() * 60.0;
    inputs.magazineAppliedVolts = magazineAppliedVolts.getValueAsDouble();
    inputs.magazineSupplyCurrentAmps = magazineSupplyCurrent.getValueAsDouble();
    inputs.magazineTorqueCurrentAmps = magazineTorqueCurrent.getValueAsDouble();
    inputs.magazineTempCelsius = magazineTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double magazineVolts) {
    magazineTalon.setControl(voltageControl.withOutput(magazineVolts));
  }

  @Override
  public void stop() {
    magazineTalon.setControl(neutralControl);
  }

  @Override
  public void runCharacterizationMagazine(double input) {
    magazineTalon.setControl(voltageControl.withOutput(input));
  }

   @Override
  public List<ParentDevice> getOrchestraDevices() {
    return List.of(magazineTalon);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Magazine Motor", magazineTalon);
  }
}
