package frc.robot.subsystems.Lift;

import org.littletonrobotics.junction.AutoLog;

import java.util.function.BooleanSupplier;

public interface ILiftIO {

    @AutoLog
    public static class LiftIOInputs {
        boolean motorConnected = false;
        double elevatorVelocity = 0.0;
        double elevatorPosition = 0.0;
        double elevatorAppliedVolts = 0.0;
        boolean foldSwitch = false;
    }

    public default void updateInputs(LiftIOInputs inputs) {
    }

    public default void setSpeed(double speed) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setVoltageWithCoral(double voltage) {
    }

    public default void setGoal(double goal) {
    }

    public default BooleanSupplier atGoal() {
        return () -> false;
    }

    public default void stopMotor() {
    }

    public default void resistGravity() {
    }

    public default void resetIfPressed() {
    }

    public default void setPidValues() {
    }

    public default void setFeedForward(double voltage) {
    }

    public default boolean isPressed() {
        return false;
    }

    public default void setVoltageWithResistGravity(double voltage) {
    }

    public default double getPosition() {
        return 0;
    }

    public default void resetPID() {
    }

    public default void resetPID(double newGoal) {
    }

}
