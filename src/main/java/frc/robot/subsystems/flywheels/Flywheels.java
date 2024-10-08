// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelConstants.*;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.SmartController;
import frc.robot.util.Alert;
import frc.robot.util.LinearProfile;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.subsystem.AdvancedSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends AdvancedSubsystem {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", gains.kD());
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", gains.kS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", gains.kV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", gains.kA());

  private static final LoggedTunableNumber prepareShootMultiplier =
      new LoggedTunableNumber("Flywheels/PrepareShootMultiplier", 1.0);
  private static final LoggedTunableNumber intakingRpm =
      new LoggedTunableNumber("Flywheels/IntakingRpm", -3000.0);
  private static final LoggedTunableNumber demoIntakingRpm =
      new LoggedTunableNumber("Flywheels/DemoIntakingRpm", -250.0);
  private static final LoggedTunableNumber ejectingRpm =
      new LoggedTunableNumber("Flywheels/EjectingRpm", 1000.0);
  private static final LoggedTunableNumber poopingRpm =
      new LoggedTunableNumber("Flywheels/PoopingRpm", 3000.0);
  private static final LoggedTunableNumber testingLowRpm =
      new LoggedTunableNumber("Flywheels/TestingLowRpm", 1000.0);
  private static final LoggedTunableNumber testingHighRpm =
      new LoggedTunableNumber("Flywheels/TestingHighRpm", 5000.0);

  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Flywheels/MaxAccelerationRpmPerSec", flywheelConfig.maxAcclerationRpmPerSec());

  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  private final LinearProfile leftProfile;
  private final LinearProfile rightProfile;
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
  private boolean wasClosedLoop = false;
  private boolean closedLoop = false;
  @Setter private BooleanSupplier prepareShootSupplier = () -> false;

  // Disconnected alerts
  private final Alert leftDisconnected =
      new Alert("Left flywheel disconnected!", Alert.AlertType.WARNING);
  private final Alert rightDisconnected =
      new Alert("Right flywheel disconnected!", Alert.AlertType.WARNING);

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    INTAKE(intakingRpm, intakingRpm),
    DEMO_INTAKE(demoIntakingRpm, demoIntakingRpm),
    EJECT(ejectingRpm, ejectingRpm),
    POOP(poopingRpm, poopingRpm),
    TESTLOW(testingLowRpm, testingLowRpm),
    TESTHIGH(testingHighRpm, testingHighRpm),
    SHOOT(
        () -> SmartController.getInstance().getTargetAimingParameters().shooterSpeed(),
        () -> SmartController.getInstance().getTargetAimingParameters().shooterSpeed()),
    CHARACTERIZING(() -> 0.0, () -> 0.0);

    private final DoubleSupplier leftGoal;
    private final DoubleSupplier rightGoal;

    private double getLeftGoal() {
      return leftGoal.getAsDouble();
    }

    private double getRightGoal() {
      return rightGoal.getAsDouble();
    }
  }

  @Getter
  @AutoLogOutput(key = "Flywheels/Goal")
  private Goal goal = Goal.IDLE;

  public Flywheels(FlywheelsIO io) {
    this.io = io;

    leftProfile = new LinearProfile(maxAcceleration.get(), Constants.loopPeriodSecs);
    rightProfile = new LinearProfile(maxAcceleration.get(), Constants.loopPeriodSecs);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Flywheels Idle"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);

    // Set alerts
    leftDisconnected.set(!inputs.leftMotorConnected);
    rightDisconnected.set(!inputs.rightMotorConnected);

    // Check controllers
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), kSVA -> ff = new SimpleMotorFeedforward(kSVA[0], kSVA[1], kSVA[2]), kS, kV, kA);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          leftProfile.setMaxAcceleration(maxAcceleration.get());
          rightProfile.setMaxAcceleration(maxAcceleration.get());
        },
        maxAcceleration);

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    // Check if profile needs to be reset
    if (!closedLoop && wasClosedLoop) {
      leftProfile.reset();
      rightProfile.reset();
      wasClosedLoop = false;
    }

    // Get goal
    double leftGoal = goal.getLeftGoal();
    double rightGoal = goal.getRightGoal();
    boolean idlePrepareShoot = goal == Goal.IDLE && prepareShootSupplier.getAsBoolean();
    if (idlePrepareShoot) {
      leftGoal = Goal.SHOOT.getLeftGoal() * prepareShootMultiplier.get();
      rightGoal = Goal.SHOOT.getRightGoal() * prepareShootMultiplier.get();
    }

    // Run to setpoint
    if (closedLoop || idlePrepareShoot) {
      // Update goals
      leftProfile.setGoal(leftGoal);
      rightProfile.setGoal(rightGoal);
      double leftSetpoint = leftProfile.calculateSetpoint();
      double rightSetpoint = rightProfile.calculateSetpoint();
      io.runVelocity(
          leftSetpoint, rightSetpoint, ff.calculate(leftSetpoint), ff.calculate(rightSetpoint));
    } else if (goal == Goal.IDLE) {
      io.stop();
    }

    Logger.recordOutput("Flywheels/SetpointLeftRpm", leftProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/SetpointRightRpm", rightProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/GoalLeftRpm", leftGoal);
    Logger.recordOutput("Flywheels/GoalRightRpm", rightGoal);
  }

  /** Set the current goal of the flywheel */
  private void setGoal(Goal goal) {
    if (goal == Goal.CHARACTERIZING || goal == Goal.IDLE) {
      wasClosedLoop = closedLoop;
      closedLoop = false;
      this.goal = goal;
      return; // Don't set a goal
    }
    // If not already controlling to requested goal
    // set closed loop false
    closedLoop = this.goal == goal;
    // Enable close loop
    if (!closedLoop) {
      leftProfile.setGoal(goal.getLeftGoal(), inputs.leftVelocityRpm);
      rightProfile.setGoal(goal.getRightGoal(), inputs.rightVelocityRpm);
      closedLoop = true;
    }
    this.goal = goal;
  }

  /** Runs flywheels at the commanded voltage or amps. */
  public void runCharacterization(double input) {
    setGoal(Goal.CHARACTERIZING);
    io.runCharacterizationLeft(input);
    io.runCharacterizationRight(input);
  }

  /** Get characterization velocity */
  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRpm + inputs.rightVelocityRpm) / 2.0;
  }

  public double getTargetRPMLeft() {
    return goal.getLeftGoal();
  }

  public double getTargetRPMRight() {
    return goal.getRightGoal();
  }

  /** Get if velocity profile has ended */
  @AutoLogOutput(key = "Flywheels/AtGoal")
  public boolean atGoal() {
    return goal == Goal.IDLE
        || (leftProfile.getCurrentSetpoint() == goal.getLeftGoal()
            && rightProfile.getCurrentSetpoint() == goal.getRightGoal());
  }

  public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Shoot");
  }

  public Command intakeCommand() {
    return startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Intake");
  }

  public Command demoIntakeCommand() {
    return startEnd(() -> setGoal(Goal.DEMO_INTAKE), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Demo Intake");
  }

  public Command ejectCommand() {
    return startEnd(() -> setGoal(Goal.EJECT), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Eject");
  }

  public Command poopCommand() {
    return startEnd(() -> setGoal(Goal.POOP), () -> setGoal(Goal.IDLE)).withName("Flywheels Poop");
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return io.getOrchestraDevices();
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
            runOnce(this::clearFaults),
            run(() -> setGoal(Goal.TESTLOW)).withTimeout(1.0),
            runOnce(
                () -> {
                  if (inputs.leftVelocityRpm < 1000) {
                    addFault("[System Check] Left shooter RPM measured too low", false, true);
                  }
                  if (inputs.rightVelocityRpm < 1000) {
                    addFault("[System Check] Right shooter RPM measured too low", false, true);
                  }
                }),
            run(() -> setGoal(Goal.TESTHIGH)).withTimeout(2.0),
            runOnce(
                () -> {
                  if (inputs.leftVelocityRpm < getTargetRPMLeft() - 300
                      || inputs.leftVelocityRpm > getTargetRPMLeft() + 300) {
                    addFault("[System Check] Left shooter did not reach target RPM", false, true);
                  }
                  if (inputs.rightVelocityRpm < getTargetRPMRight() - 300
                      || inputs.rightVelocityRpm > getTargetRPMRight() + 300) {
                    addFault("[System Check] Right shooter did not reach target RPM", false, true);
                  }
                }))
        .until(() -> !getFaults().isEmpty())
        .andThen(runOnce(() -> setGoal(Goal.IDLE)));
  }
}
