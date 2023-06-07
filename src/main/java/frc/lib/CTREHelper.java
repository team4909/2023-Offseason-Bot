package frc.lib;

import java.util.Map;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.Logger.LoggedMap;

public class CTREHelper {

  public static LoggedMap<Double> getDeviceLog(TalonFX talonFX, String subTable) {
    return new LoggedMap<>(Map.ofEntries(
        Map.entry(new LoggedDouble(subTable, "Position"), talonFX.getPosition().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Velocity"), talonFX.getVelocity().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Rotor Position"), talonFX.getRotorPosition().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Rotor Velocity"), talonFX.getRotorVelocity().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Closed Loop Reference"), talonFX.getClosedLoopReference().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Closed Loop Error"), talonFX.getClosedLoopError().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Torque Current"), talonFX.getTorqueCurrent().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Stator Current"), talonFX.getStatorCurrent().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Supply Current"), talonFX.getSupplyCurrent().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Supply Voltage"), talonFX.getSupplyVoltage().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Device Temperature"), talonFX.getDeviceTemp().asSupplier())));
  }

  public static LoggedMap<Double> getDeviceLog(CANcoder cancoder, String subTable) {
    return new LoggedMap<>(Map.ofEntries(
        Map.entry(new LoggedDouble(subTable, "Position"), cancoder.getPosition().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Velocity"), cancoder.getVelocity().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Absolute Position"), cancoder.getAbsolutePosition().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Unfiltered Velocity"), cancoder.getUnfilteredVelocity().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Displacement Since Boot"),
            cancoder.getPositionSinceBoot().asSupplier())));
  }

  public static LoggedMap<Double> getDeviceLog(Pigeon2 pigeon, String subTable) {
    return new LoggedMap<>(Map.ofEntries(
        Map.entry(new LoggedDouble(subTable, "Up Time"), pigeon.getUpTime().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Yaw"), pigeon.getYaw().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Pitch"), pigeon.getPitch().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Roll"), pigeon.getRoll().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Supply Voltage"), pigeon.getSupplyVoltage().asSupplier()),
        Map.entry(new LoggedDouble(subTable, "Device Temperature"), pigeon.getTemperature().asSupplier())));
  }

  public static class StatusLogger {

  }
}
