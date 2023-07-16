package frc.lib;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class Logger {

  private static Logger m_instance;
  private final NetworkTableInstance NT = NetworkTableInstance.getDefault();
  private boolean seenFMSCache = false;
  public final NetworkTable logTable = NT.getTable("Robot");
  public final BooleanSupplier useNT = () -> !getCachedFMSPresence();

  private boolean getCachedFMSPresence() {
    if (seenFMSCache)
      return true;
    else if (DriverStation.isFMSAttached()) {
      seenFMSCache = true;
      return true;
    } else {
      return false;
    }
  }

  public record CTRESignalMap<T>(
      Map<String, StatusSignal<T>> map) {
  }

  public static Logger getInstance() {
    return m_instance = (m_instance == null) ? new Logger() : m_instance;
  }
}
