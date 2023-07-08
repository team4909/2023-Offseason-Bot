package frc.lib;

import java.util.Map;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.Logger.CTRESignalMap;

public class CTREHelper {

  public static CTRESignalMap<Double> getDeviceLog(TalonFX talonFX, String subTable) {
    return new CTRESignalMap<>(Map.ofEntries(
        Map.entry(new LoggedDouble(subTable, "Position"), talonFX.getPosition()),
        Map.entry(new LoggedDouble(subTable, "Velocity"), talonFX.getVelocity()),
        Map.entry(new LoggedDouble(subTable, "Rotor Position"), talonFX.getRotorPosition()),
        Map.entry(new LoggedDouble(subTable, "Rotor Velocity"), talonFX.getRotorVelocity()),
        Map.entry(new LoggedDouble(subTable, "Closed Loop Reference"), talonFX.getClosedLoopReference()),
        Map.entry(new LoggedDouble(subTable, "Closed Loop Error"), talonFX.getClosedLoopError()),
        Map.entry(new LoggedDouble(subTable, "Torque Current"), talonFX.getTorqueCurrent()),
        Map.entry(new LoggedDouble(subTable, "Stator Current"), talonFX.getStatorCurrent()),
        Map.entry(new LoggedDouble(subTable, "Supply Current"), talonFX.getSupplyCurrent()),
        Map.entry(new LoggedDouble(subTable, "Supply Voltage"), talonFX.getSupplyVoltage()),
        Map.entry(new LoggedDouble(subTable, "Device Temperature"), talonFX.getDeviceTemp())));
  }

  public static CTRESignalMap<Double> getDeviceLog(CANcoder cancoder, String subTable) {
    return new CTRESignalMap<>(Map.ofEntries(
        Map.entry(new LoggedDouble(subTable, "Position"), cancoder.getPosition()),
        Map.entry(new LoggedDouble(subTable, "Velocity"), cancoder.getVelocity()),
        Map.entry(new LoggedDouble(subTable, "Absolute Position"), cancoder.getAbsolutePosition()),
        Map.entry(new LoggedDouble(subTable, "Unfiltered Velocity"), cancoder.getUnfilteredVelocity()),
        Map.entry(new LoggedDouble(subTable, "Displacement Since Boot"),
            cancoder.getPositionSinceBoot())));
  }

  public static CTRESignalMap<Double> getDeviceLog(Pigeon2 pigeon, String subTable) {
    return new CTRESignalMap<>(Map.ofEntries(
        Map.entry(new LoggedDouble(subTable, "Up Time"), pigeon.getUpTime()),
        Map.entry(new LoggedDouble(subTable, "Yaw"), pigeon.getYaw()),
        Map.entry(new LoggedDouble(subTable, "Pitch"), pigeon.getPitch()),
        Map.entry(new LoggedDouble(subTable, "Roll"), pigeon.getRoll()),
        Map.entry(new LoggedDouble(subTable, "Supply Voltage"), pigeon.getSupplyVoltage()),
        Map.entry(new LoggedDouble(subTable, "Device Temperature"), pigeon.getTemperature())));
  }

  public static class StatusLogger {

  }
}
