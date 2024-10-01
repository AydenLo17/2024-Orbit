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

import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.util.subsystem.AdvancedSubsystem;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeMotorConnected = true;
    public double intakePositionRads = 0.0;
    public double intakeVelocityRpm = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeSupplyCurrentAmps = 0.0;
    public double intakeTorqueCurrentAmps = 0.0;
    public double intakeTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run intake at volts */
  default void runVolts(double volts) {}

  /** Stop intake */
  public default void stop() {}

  /** Run intake at voltage */
  default void runCharacterizationIntake(double input) {}

  default List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }

  default void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}
}
