// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.util.subsystem.AdvancedSubsystem;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
  @AutoLog
  class FlywheelsIOInputs {
    public boolean leftMotorConnected = true;
    public boolean rightMotorConnected = true;

    public double leftPositionRads = 0.0;
    public double leftVelocityRpm = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTorqueCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;

    public double rightPositionRads = 0.0;
    public double rightVelocityRpm = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTorqueCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;
  }

  /** Update inputs */
  default void updateInputs(FlywheelsIOInputs inputs) {}

  /** Run both motors at voltage */
  default void runVolts(double leftVolts, double rightVolts) {}

  /** Stop both flywheels */
  default void stop() {}

  /** Run left and right flywheels at velocity in rpm */
  default void runVelocity(
      double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward) {}

  /** Config PID values for both motors */
  default void setPID(double kP, double kI, double kD) {}

  /** Config FF values for both motors */
  default void setFF(double kS, double kV, double kA) {}

  /** Run left flywheels at voltage */
  default void runCharacterizationLeft(double input) {}

  /** Run right flywheels at voltage */
  default void runCharacterizationRight(double input) {}

  default List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }

  default void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}
}
