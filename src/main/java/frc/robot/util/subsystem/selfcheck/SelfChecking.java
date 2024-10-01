package frc.robot.util.subsystem.selfcheck;

import frc.robot.util.subsystem.SubsystemFault;
import java.util.List;

public interface SelfChecking {
  List<SubsystemFault> checkForFaults();
}
