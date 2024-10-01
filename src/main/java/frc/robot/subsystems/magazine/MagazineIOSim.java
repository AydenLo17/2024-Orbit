// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.magazine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class MagazineIOSim implements MagazineIO {
  private final FlywheelSim magazineSim = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 3,
      0.00363458292);

  private double leftAppliedVolts = 0.0;

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    magazineSim.update(Constants.loopPeriodSecs);

    inputs.magazinePositionRads += Units
        .radiansToRotations(magazineSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
    inputs.magazineVelocityRpm = magazineSim.getAngularVelocityRPM();
    inputs.magazineAppliedVolts = leftAppliedVolts;
    inputs.magazineSupplyCurrentAmps = magazineSim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double leftVolts) {
    leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    magazineSim.setInputVoltage(leftAppliedVolts);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }

  @Override
  public void runCharacterizationMagazine(double input) {
    runVolts(input);
  }
}