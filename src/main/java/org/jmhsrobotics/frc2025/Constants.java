package org.jmhsrobotics.frc2025;

import edu.wpi.first.wpilibj.RobotBase;
import org.jmhsrobotics.warcore.util.NetworkUtil.MACAddress;
import org.jmhsrobotics.warcore.util.RobotIdentifier;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum RobotType {
    TEST_BOT,
    HARLEY,
    COMP_BOT,
  }

  private static RobotIdentifier<RobotType> ident;

  static {
    ident = new RobotIdentifier<RobotType>();
    ident.addDefaultRobot(RobotType.COMP_BOT);
    ident.addRobot(
        new MACAddress("DE:AD:BE:EF:C0:DE"), RobotType.COMP_BOT); // TODO: Get Real Mac Address
    ident.addRobot(
        new MACAddress("DE:AD:BE:EF:C1:DE"), RobotType.TEST_BOT); // TODO: Get Real Mac Address
    ident.addRobot(
        new MACAddress("DE:AD:BE:EF:C2:DE"), RobotType.HARLEY); // TODO: Get Real Mac Address
  }

  public static RobotType ROBOT_TYPE = ident.identify();
}
