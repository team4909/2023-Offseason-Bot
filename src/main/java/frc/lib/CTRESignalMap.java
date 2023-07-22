package frc.lib;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;

public record CTRESignalMap<T>(
    String name, ArrayList<StatusSignal<T>> signals) {
}