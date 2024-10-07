// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.SmartController;
import frc.robot.util.Alert;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.NoteVisualizer;
import frc.robot.util.subsystem.AdvancedSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends AdvancedSubsystem {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Arm/Gains/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Arm/Gains/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Arm/Gains/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Arm/Gains/kG", gains.ffkG());
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Arm/Velocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Arm/Acceleration", profileConstraints.maxAcceleration);
  private static final LoggedTunableNumber smoothVelocity =
      new LoggedTunableNumber("Arm/SmoothVelocity", profileConstraints.maxVelocity * 0.75);
  private static final LoggedTunableNumber smoothAcceleration =
      new LoggedTunableNumber("Arm/SmoothAcceleration", profileConstraints.maxAcceleration * 0.5);
  private static final LoggedTunableNumber prepareClimbVelocity =
      new LoggedTunableNumber("Arm/PrepareClimbVelocity", 1.5);
  private static final LoggedTunableNumber prepareClimbAcceleration =
      new LoggedTunableNumber("Arm/PrepareClimbAcceleration", 2.5);
  private static final LoggedTunableNumber lowerLimitDegrees =
      new LoggedTunableNumber("Arm/LowerLimitDegrees", minAngle.getDegrees());
  private static final LoggedTunableNumber upperLimitDegrees =
      new LoggedTunableNumber("Arm/UpperLimitDegrees", maxAngle.getDegrees());
  private static final LoggedTunableNumber partialStowUpperLimitDegrees =
      new LoggedTunableNumber("Arm/PartialStowUpperLimitDegrees", 30.0);
  private static final LoggedTunableNumber closeShootingZoneFeet =
      new LoggedTunableNumber("RobotState/CloseShootingZoneFeet", 10.0);

  // Profile constraints
  public static final Supplier<TrapezoidProfile.Constraints> maxProfileConstraints =
      () -> new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
  public static final Supplier<TrapezoidProfile.Constraints> smoothProfileConstraints =
      () -> new TrapezoidProfile.Constraints(smoothVelocity.get(), smoothAcceleration.get());
  public static final Supplier<TrapezoidProfile.Constraints> prepareClimbProfileConstraints =
      () ->
          new TrapezoidProfile.Constraints(
              prepareClimbVelocity.get(), prepareClimbAcceleration.get());

  @RequiredArgsConstructor
  public enum Goal {
    STOW(() -> 0),
    UNJAM_INTAKE(new LoggedTunableNumber("Arm/UnjamDegrees", 40.0)),
    STATION_INTAKE(new LoggedTunableNumber("Arm/StationIntakeDegrees", 45.0)),
    AIM(
        () ->
            SmartController.getInstance().getTargetAimingParameters().shooterAngle().getDegrees()),
    DEMO_SHOT(new LoggedTunableNumber("Arm/DemoShotDegrees", 40.0)),
    AMP(new LoggedTunableNumber("Arm/AmpDegrees", 110.0)),
    SUBWOOFER(new LoggedTunableNumber("Arm/SubwooferDegrees", 55.0)),
    PODIUM(new LoggedTunableNumber("Arm/PodiumDegrees", 34.0)),
    CLIMB(new LoggedTunableNumber("Arm/ClimbDegrees", 88.0)),
    TEST(new LoggedTunableNumber("Arm/CustomSetpoint", 55.0)),
    CUSTOM(new LoggedTunableNumber("Arm/CustomSetpoint", 20.0));

    private final DoubleSupplier armSetpointSupplier;

    private double getDegrees() {
      return armSetpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput @Getter @Setter private Goal goal = Goal.STOW;
  private boolean characterizing = false;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  @AutoLogOutput @Setter private double currentCompensation = 0.0;
  private TrapezoidProfile.Constraints currentConstraints = maxProfileConstraints.get();
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private double goalAngle;
  private ArmFeedforward ff;

  private final ArmVisualizer visualizerMeasured;
  private final ArmVisualizer visualizerSetpoint;
  private final ArmVisualizer goalVisualizer;

  private final Alert leaderMotorDisconnected =
      new Alert("Arm leader motor disconnected!", Alert.AlertType.WARNING);
  private final Alert followerMotorDisconnected =
      new Alert("Arm follower motor disconnected!", Alert.AlertType.WARNING);
  private final Alert absoluteEncoderDisconnected =
      new Alert("Arm absolute encoder disconnected!", Alert.AlertType.WARNING);

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;
  private BooleanSupplier halfStowSupplier = () -> true;
  private boolean brakeModeEnabled = true;

  private boolean wasNotAuto = false;

  public Arm(ArmIO io) {
    this.io = io;
    io.setBrakeMode(true);

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    io.setPID(kP.get(), kI.get(), kD.get());
    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    // Set up visualizers
    NoteVisualizer.setArmAngleSupplier(() -> Rotation2d.fromRadians(inputs.positionDegrees));
    visualizerMeasured = new ArmVisualizer("ArmMeasured", null);
    visualizerSetpoint = new ArmVisualizer("ArmSetpoint", new Color8Bit(Color.kOrange));
    goalVisualizer = new ArmVisualizer("Goal", new Color8Bit(Color.kBlue));
  }

  public void setOverrides(
      BooleanSupplier disableOverride,
      BooleanSupplier coastOverride,
      BooleanSupplier halfStowOverride) {
    disableSupplier = () -> disableOverride.getAsBoolean() || DriverStation.isDisabled();
    coastSupplier = coastOverride;
    halfStowSupplier = halfStowOverride;
  }

  private double getStowAngle() {
    if (DriverStation.isTeleopEnabled()
        && SmartController.getInstance().getPrerollDistance() < closeShootingZoneFeet.get()
        && !halfStowSupplier.getAsBoolean()) {
      return MathUtil.clamp(
          setpointState.position,
          minAngle.getRadians(),
          Units.degreesToRadians(partialStowUpperLimitDegrees.get()));
    } else {
      return minAngle.getRadians();
    }
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Set alerts
    leaderMotorDisconnected.set(!inputs.leaderMotorConnected);
    followerMotorDisconnected.set(!inputs.followerMotorConnected);
    absoluteEncoderDisconnected.set(!inputs.absoluteEncoderConnected);

    // Update controllers
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);

    // Check if disabled
    // Also run first cycle of auto to reset arm
    if (disableSupplier.getAsBoolean()
        || (Constants.getMode() == Constants.Mode.SIM
            && DriverStation.isAutonomousEnabled()
            && wasNotAuto)) {
      io.stop();
      // Reset profile when disabled
      setpointState = new TrapezoidProfile.State(inputs.positionDegrees, 0);
    }
    // Track autonomous enabled
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    // Set coast mode with override
    setBrakeMode(!coastSupplier.getAsBoolean());

    // Don't run profile when characterizing, coast mode, or disabled
    if (!characterizing && brakeModeEnabled && !disableSupplier.getAsBoolean()) {
      // Run closed loop
      goalAngle =
          goal.getDegrees()
              + (goal == Goal.AIM ? Units.degreesToRadians(currentCompensation) : 0.0);
      if (goal == Goal.STOW) {
        goalAngle = getStowAngle();
      }
      setpointState =
          profile.calculate(
              Constants.loopPeriodSecs,
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      goalAngle,
                      Units.degreesToRadians(lowerLimitDegrees.get()),
                      Units.degreesToRadians(upperLimitDegrees.get())),
                  0.0));
      if (goal == Goal.STOW
          && EqualsUtil.epsilonEquals(goalAngle, minAngle.getRadians())
          && atGoal()) {
        io.stop();
      } else {
        io.runSetpoint(
            setpointState.position, ff.calculate(setpointState.position, setpointState.velocity));
      }

      goalVisualizer.update(goalAngle);
      Logger.recordOutput("Arm/GoalAngle", goalAngle);
    }

    // Logs
    visualizerMeasured.update(inputs.positionDegrees);
    visualizerSetpoint.update(setpointState.position);
    Logger.recordOutput("Arm/SetpointAngle", setpointState.position);
    Logger.recordOutput("Arm/SetpointVelocity", setpointState.velocity);
    Logger.recordOutput("Arm/Goal", goal);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Arm/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(setpointState.position, goalAngle, 1e-3);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void setProfileConstraints(TrapezoidProfile.Constraints constraints) {
    if (EqualsUtil.epsilonEquals(currentConstraints.maxVelocity, constraints.maxVelocity)
        && EqualsUtil.epsilonEquals(currentConstraints.maxAcceleration, constraints.maxVelocity))
      return;
    currentConstraints = constraints;
    profile = new TrapezoidProfile(currentConstraints);
  }

  public void runCharacterization(double amps) {
    characterizing = true;
    io.runCurrent(amps);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityDegreesPerSec;
  }

  public void endCharacterization() {
    characterizing = false;
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return io.getOrchestraDevices();
  }

  public double getArmAngle() {
    return inputs.positionDegrees;
  }

  public Transform3d getFlywheelPosition() {
    return new Transform3d(
        new Pose3d() {}, visualizerMeasured.getArmPose(inputs.absoluteEncoderPositionDegrees));
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
            runOnce(this::clearFaults),
            run(() -> setGoal(Goal.TEST)).withTimeout(5.0),
            runOnce(
                () -> {
                  if (inputs.positionDegrees < getGoal().getDegrees() - 1
                      || inputs.positionDegrees > getGoal().getDegrees() + 1) {
                    addFault("[System Check] Arm  did not reach target TEST", false, true);
                  }
                }),
            run(() -> setGoal(Goal.AMP)).withTimeout(5.0),
            runOnce(
                () -> {
                  if (inputs.positionDegrees < getGoal().getDegrees() - 1
                      || inputs.positionDegrees > getGoal().getDegrees() + 1) {
                    addFault("[System Check] Arm  did not reach target AMP", false, true);
                  }
                }))
        .until(() -> !getFaults().isEmpty())
        .andThen(runOnce(() -> setGoal(Goal.STOW)));
  }
}
