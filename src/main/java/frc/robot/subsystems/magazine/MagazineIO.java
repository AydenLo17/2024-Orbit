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

import java.util.Collections;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.ParentDevice;

import frc.robot.util.subsystem.AdvancedSubsystem;

public interface MagazineIO {
  @AutoLog
  public static class MagazineIOInputs {
    public boolean magazineMotorConnected = true;
    public double magazinePositionRads = 0.0;
    public double magazineVelocityRpm = 0.0;
    public double magazineAppliedVolts = 0.0;
    public double magazineSupplyCurrentAmps = 0.0;
    public double magazineTorqueCurrentAmps = 0.0;
    public double magazineTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(MagazineIOInputs inputs) {
  }

  /** Run magazine at volts */
  default void runVolts(double volts) {
  }

  /** Stop magazine */
  public default void stop() {
  }

    /** Run magazine at voltage */
    default void runCharacterizationMagazine(double input) {
    }

  default List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }

  default void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
  }
}
