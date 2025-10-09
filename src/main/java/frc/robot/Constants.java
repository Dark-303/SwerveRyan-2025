package frc.robot;

public final class Constants {
    public static final double globalDelta_s = 0.02; // 0.02 = 50Hz
    public static final double globalDelta_Hz = 1.0 / globalDelta_s; // 0.02 = 50Hz

    public enum Mode {
        REAL,
        SIM
    }
}