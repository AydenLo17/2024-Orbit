// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.util.subsystem.AdvancedSubsystem;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public boolean leaderMotorConnected = true;
    public boolean followerMotorConnected = true;

    public double positionDegrees = 0.0;
    public double absoluteEncoderPositionDegrees = 0.0;
    public double relativeEncoderPositionDegrees = 0.0;
    public double velocityDegreesPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
    public boolean absoluteEncoderConnected = true;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  /** Run to setpoint angle in radians */
  default void runSetpoint(double setpointDegrees, double feedforward) {}

  /** Run motors at volts */
  default void runVolts(double volts) {}

  /** Run motors at current */
  default void runCurrent(double amps) {}

  /** Set brake mode enabled */
  default void setBrakeMode(boolean enabled) {}

  /** Set PID values */
  default void setPID(double p, double i, double d) {}

  /** Stops motors */
  default void stop() {}

  default List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }

  default void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}
}
