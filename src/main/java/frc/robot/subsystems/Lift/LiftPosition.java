package frc.robot.subsystems.Lift;

public enum LiftPosition {
    // TODO: check what the actual height of the reefs is.
    L0(0.0), L1(0.0), L2(0.0), L3(0.0), L4(0.0);
    private double m_HeightMeters;
    private LiftPosition(double heightMeters) {
        this.m_HeightMeters = heightMeters;
    }
    public double getHeightMeters() {
        return m_HeightMeters;
    }
}
