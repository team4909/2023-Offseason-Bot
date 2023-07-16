package frc.lib;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class CTREHelper {

  public static ArrayList<StatusSignal<Double>> getRelevantSignals(TalonFX talonFX) {
    return new ArrayList<>(List.of(
        talonFX.getPosition(),
        talonFX.getVelocity(),
        talonFX.getClosedLoopReference(),
        talonFX.getClosedLoopError(),
        talonFX.getTorqueCurrent(),
        talonFX.getStatorCurrent(),
        talonFX.getSupplyCurrent(),
        talonFX.getSupplyVoltage(),
        talonFX.getDeviceTemp()));
  }

  public static ArrayList<StatusSignal<Double>> getRelevantSignals(CANcoder cancoder) {
    return new ArrayList<>(List.of(
        cancoder.getPosition(),
        cancoder.getVelocity(),
        cancoder.getAbsolutePosition(),
        cancoder.getUnfilteredVelocity(),
        cancoder.getPositionSinceBoot()));
  }
  // public static CTRESignalMap<Double> getDeviceLog(Pigeon2 pigeon, String
  // subTable) {
  // return new CTRESignalMap<>(Map.ofEntries(
  // Map.entry(new LoggedDouble(subTable, "Up Time"), pigeon.getUpTime()),
  // Map.entry(new LoggedDouble(subTable, "Yaw"), pigeon.getYaw()),
  // Map.entry(new LoggedDouble(subTable, "Pitch"), pigeon.getPitch()),
  // Map.entry(new LoggedDouble(subTable, "Roll"), pigeon.getRoll()),
  // Map.entry(new LoggedDouble(subTable, "Supply Voltage"),
  // pigeon.getSupplyVoltage()),
  // Map.entry(new LoggedDouble(subTable, "Device Temperature"),
  // pigeon.getTemperature())));
  // }

  public static void updateBulkSignals(ArrayList<?>... signalLists) {
    StatusSignal.waitForAll(0,
        Arrays.stream(signalLists)
            .flatMap(Collection::stream)
            .toArray(BaseStatusSignal[]::new));
  }
}
