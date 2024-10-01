package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;

public class CompositeCommand {
  Drive drive;
  Flywheels flywheels;
  InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

  public CompositeCommand(Drive drive, Flywheels flywheels) {
    this.drive = drive;
    this.flywheels = flywheels;

    // Populate the distance map with distance-speed pairs
    distanceMap.put(1.0, 10.0);
    distanceMap.put(2.3, 15.7);
    distanceMap.put(3.6, 21.9);
    distanceMap.put(4.9, 27.8);
    distanceMap.put(6.2, 33.6);
    distanceMap.put(7.5, 39.4);
  }

  //   public Command speakerShoot() {
  //     return Command
  // }
}
