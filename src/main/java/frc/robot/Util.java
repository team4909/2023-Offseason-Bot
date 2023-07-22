package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Util {

  public static DCMotor getFalcon500FOCGearbox(int numMotors, double reduction) {
    return new DCMotor(12, 5.84, 304, 1.5, Units.rotationsPerMinuteToRadiansPerSecond(6080.0), numMotors)
        .withReduction(reduction);
  }

  // public static void logFalconError(StatusCode statusCode, String
  // deviceIdentifier) {
  // LoggedString output = new LoggedString("API Error", deviceIdentifier);
  // if (!statusCode.equals(StatusCode.OK)) {
  // output.accept(statusCode.getName() + " " + statusCode.getDescription());
  // }
  // }
}
