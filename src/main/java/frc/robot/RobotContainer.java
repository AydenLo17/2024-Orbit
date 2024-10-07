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

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CompositeCommand;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOKrakenFOC;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIO;
import frc.robot.subsystems.flywheels.FlywheelsIOKrakenFOC;
import frc.robot.subsystems.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKrakenFOC;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.linebreak.LineBreak;
import frc.robot.subsystems.linebreak.LineBreakIO;
import frc.robot.subsystems.linebreak.LineBreakIODigitalInput;
import frc.robot.subsystems.linebreak.LineBreakIOSim;
import frc.robot.subsystems.magazine.Magazine;
import frc.robot.subsystems.magazine.MagazineIO;
import frc.robot.subsystems.magazine.MagazineIOKrakenFOC;
import frc.robot.subsystems.magazine.MagazineIOSim;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVisionSIM;
import frc.robot.util.subsystem.AdvancedSubsystem;
import frc.robot.util.visualizer.NoteVisualizer;
import frc.robot.util.visualizer.RobotGamePieceVisualizer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private static Drive drive;
  private static AprilTagVision aprilTagVision;
  private static Flywheels flywheels;
  private static Intake intake;
  private static Magazine magazine;
  private static LineBreak lineBreak;
  private static Arm arm;
  private final CompositeCommand compositeCommand;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(moduleConfigs[0]),
                new ModuleIOTalonFX(moduleConfigs[1]),
                new ModuleIOTalonFX(moduleConfigs[2]),
                new ModuleIOTalonFX(moduleConfigs[3]));
        aprilTagVision = new AprilTagVision(new AprilTagVisionIOLimelight("limelight"));
        flywheels = new Flywheels(new FlywheelsIOKrakenFOC());
        magazine = new Magazine(new MagazineIOKrakenFOC());
        intake = new Intake(new IntakeIOKrakenFOC());
        lineBreak = new LineBreak(new LineBreakIODigitalInput());
        arm = new Arm(new ArmIOKrakenFOC());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        aprilTagVision =
            new AprilTagVision(
                new AprilTagVisionIOPhotonVisionSIM(
                    "photonCamera1",
                    new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)),
                    drive::getDrive));
        flywheels = new Flywheels(new FlywheelsIOSim());
        magazine = new Magazine(new MagazineIOSim());
        intake = new Intake(new IntakeIOSim());
        lineBreak = new LineBreak(new LineBreakIOSim());
        arm = new Arm(new ArmIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
        flywheels = new Flywheels(new FlywheelsIO() {});
        magazine = new Magazine(new MagazineIO() {});
        intake = new Intake(new IntakeIO() {});
        lineBreak = new LineBreak(new LineBreakIO() {});
        arm = new Arm(new ArmIO() {});

        break;
    }

    NoteVisualizer.setRobotPoseSupplier(
        () ->
            new Pose3d(
                new Translation3d(
                    drive.getPose().getTranslation().getX(),
                    drive.getPose().getTranslation().getY(),
                    0),
                new Rotation3d(0, 0, drive.getPose().getRotation().getRadians())));

    RobotGamePieceVisualizer.setRobotPoseSupplier(
        () ->
            new Pose3d(
                new Translation3d(
                    drive.getPose().getTranslation().getX(),
                    drive.getPose().getTranslation().getY(),
                    0),
                new Rotation3d(0, 0, drive.getPose().getRotation().getRadians())));

    RobotGamePieceVisualizer.setArmTransformSupplier(arm::getFlywheelPosition);
    RobotGamePieceVisualizer.setShooterAngleSupplier(arm::getArmAngle);
    RobotGamePieceVisualizer.setIsMagazineLoadedSupplier(lineBreak::hasGamePieceIntake);
    RobotGamePieceVisualizer.setIsShooterLoadedSupplier(lineBreak::isShooterLoaded);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    aprilTagVision.setDataInterfaces(drive::addVisionData);
    compositeCommand = new CompositeCommand(drive, flywheels);
    addNTCommands();
    configureAutos();
    configureButtonBindings();
  }

  private void configureAutos() {
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    autoChooser.addOption(
        "Wheel Radius Characterization",
        Commands.run(drive::setWheelsToCircle)
            .withTimeout(2)
            .andThen(new WheelRadiusCharacterization(drive)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        drive
            .swerveDriveCommand(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX())
            .withName("Drive Teleop Input"));

    // controller.a().whileTrue(compositeCommand.speakerShoot().withName("Speaker
    // Smart Shooting"));
  }

  private static void addNTCommands() {
    SmartDashboard.putData("SystemStatus/AllSystemsCheck", allSystemsCheck());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static Command allSystemsCheck() {
    return Commands.sequence(
        drive.getSystemCheckCommand(),
        flywheels.getSystemCheckCommand(),
        intake.getSystemCheckCommand(),
        magazine.getSystemCheckCommand(),
        arm.getSystemCheckCommand());
  }

  public static boolean allSystemsOK() {
    return drive.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && flywheels.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && intake.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && magazine.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && arm.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK;
  }
}
