package frc.robot;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.LoggedString;

public class Util {

  public static DCMotor getFalcon500FOC(int numMotors) {
    return new DCMotor(12, 5.84, 304, 1.5, Units.rotationsPerMinuteToRadiansPerSecond(6080.0), numMotors);
  }

  public static void logFalconError(StatusCode statusCode, String deviceIdentifier) {
    LoggedString output = new LoggedString("API Error", deviceIdentifier);
    if (!statusCode.equals(StatusCode.OK)) {
      output.accept(statusCode.getName() + " " + statusCode.getDescription());
    }
  }
}
