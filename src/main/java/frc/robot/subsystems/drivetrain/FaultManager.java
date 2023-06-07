package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.LoggedStringArray;

public class FaultManager {

  private static FaultManager m_instance;
  private final HashMap<TalonFX, HashMap<String, StatusSignal<Boolean>>> m_talonMap = new HashMap<>();
  private final ArrayList<String> faults = new ArrayList<>();
  private final LoggedStringArray loggedFaults = new LoggedStringArray("", "Sticky Faults");

  private HashMap<String, StatusSignal<Boolean>> getTalonStickyFaultSignals(TalonFX talon) {
    return new HashMap<>(Map.ofEntries(
        Map.entry("BootDuringEnable", talon.getStickyFault_BootDuringEnable()),
        Map.entry("DeviceTemp", talon.getStickyFault_DeviceTemp()),
        Map.entry("ForwardHardLimit", talon.getStickyFault_ForwardHardLimit()),
        Map.entry("ForwardSoftLimit", talon.getStickyFault_ForwardSoftLimit()),
        Map.entry("FusedSensorOutOfSync", talon.getStickyFault_FusedSensorOutOfSync()),
        Map.entry("Hardware", talon.getStickyFault_Hardware()),
        Map.entry("MissingRemoteSensor", talon.getStickyFault_MissingRemoteSensor()),
        Map.entry("OverSupplyV", talon.getStickyFault_OverSupplyV()),
        Map.entry("ProcTemp", talon.getStickyFault_ProcTemp()),
        Map.entry("ReverseHardLimit", talon.getStickyFault_ReverseHardLimit()),
        Map.entry("ReverseSoftLimit", talon.getStickyFault_ReverseSoftLimit()),
        Map.entry("StatorCurrLimit", talon.getStickyFault_StatorCurrLimit()),
        Map.entry("SupplyCurrLimit", talon.getStickyFault_SupplyCurrLimit()),
        Map.entry("Undervoltage", talon.getStickyFault_Undervoltage()),
        Map.entry("UnstableSupplyV", talon.getStickyFault_UnstableSupplyV())));
  }

  private void updateNetworkTablesEntry(String faultInfo) {
    if (!faults.contains(faultInfo))
      faults.add(faultInfo);
    loggedFaults.accept(faults.toArray(String[]::new));
  }

  public void addTalon(TalonFX talon) {
    m_talonMap.put(talon, getTalonStickyFaultSignals(talon));
  }

  public void poll() {
    m_talonMap.forEach((talon, faultList) -> {
      StatusCode status = BaseStatusSignal.waitForAll(0, faultList.values().toArray(BaseStatusSignal[]::new));
      if (!status.equals(StatusCode.OK))
        System.err.println(status);
      faultList.forEach((faultName, faultSignal) -> {
        if (faultSignal.getValue() == true)
          updateNetworkTablesEntry(talon.getCANBus() + ": " + talon.getDeviceID() + " - " + faultName);
      });
    });
  }

  public static FaultManager getInstance() {
    return m_instance = (m_instance == null) ? new FaultManager() : m_instance;
  }
}
