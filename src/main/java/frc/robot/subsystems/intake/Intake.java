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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Alert;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.subsystem.AdvancedSubsystem;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;

public class Intake extends AdvancedSubsystem {
  private static final LoggedTunableNumber intakingVolts = new LoggedTunableNumber("Intake/IntakingVolts", 12.0);
  private static final LoggedTunableNumber ejectingVolts = new LoggedTunableNumber("Intake/EjectingVolts", -12.0);

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Disconnected alerts
  private final Alert intakeDisconnected = new Alert("Intake motor disconnected!", Alert.AlertType.WARNING);

  public enum Goal {
    IDLE(() -> 0.0),
    INTAKE(intakingVolts),
    EJECT(ejectingVolts),
    CHARACTERIZING(() -> 0.0);

    private final DoubleSupplier intakeGoal;

    Goal(DoubleSupplier intakeGoal) {
      this.intakeGoal = intakeGoal;
    }

    private double getIntakeGoal() {
      return intakeGoal.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Intake/Goal")
  private Goal goal = Goal.IDLE;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    setDefaultCommand(runOnce(io::stop).andThen(run(() -> {
    })).withName("Intake Idle"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Set alerts
    intakeDisconnected.set(!inputs.intakeMotorConnected);

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    // Get goal
    double intakeGoal = goal.getIntakeGoal();
    Logger.recordOutput("Intake/Goal", intakeGoal);
  }

  /** Set the current goal of the intake */
  private void setGoal(Goal goal) {
    if (goal == Goal.CHARACTERIZING || goal == Goal.IDLE) {
      this.goal = goal;
      return; // Don't set a goal
    }
  }

  /** Runs intake at the commanded voltage or amps. */
  public void runCharacterization(double input) {
    setGoal(Goal.CHARACTERIZING);
    io.runCharacterizationIntake(input);
  }

  /** Gets intake characterization velocity */
  public double getCharacterizationVelocity() {
    return (inputs.intakeVelocityRpm);
  }

  public double getTargetVolts() {
    return goal.getIntakeGoal();
  }

  public Command intakeCommand() {
    return startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.IDLE))
        .withName("Intake Intake");
  }

  public Command ejectCommand() {
    return startEnd(() -> setGoal(Goal.EJECT), () -> setGoal(Goal.IDLE))
        .withName("Intake Eject");
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return io.getOrchestraDevices();
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
        runOnce(this::clearFaults),
        run(() -> runCharacterization(8.0)).withTimeout(1.0),
        runOnce(
            () -> {
              if (getCharacterizationVelocity() < 1000) {
                addFault("[System Check] Intake RPM measured too low", false, true);
              }
            }))

        .until(() -> !getFaults().isEmpty())
        .andThen(runOnce(() -> runCharacterization(0)));
  }
}
