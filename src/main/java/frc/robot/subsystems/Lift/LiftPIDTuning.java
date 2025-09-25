package frc.robot.subsystems.Lift;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LiftPIDTuning
{
    public double getkP() {
        return kP.get();
    }

    public double getkI() {
        return kI.get();
    }

    public double getkD() {
        return kD.get();
    }

    public double getkV() {
        return kV.get();
    }

    public double getkG() {
        return kG.get();
    }

    public double getkS() {
        return kS.get();
    }

    public double getMaxAcceleration() {
        return MaxAcceleration.get();
    }

    public double getMaxVelocity() {
        return MaxVelocity.get();
    }

    public double getUpperKG() {
        return UpperKG.get();
    }

    public double getkGWithCoral() {
        return kGWithCoral.get();
    }

    LoggedNetworkNumber kP;
    LoggedNetworkNumber kI;
    LoggedNetworkNumber kD;
    LoggedNetworkNumber kV;
    LoggedNetworkNumber kG;
    LoggedNetworkNumber kS;
    LoggedNetworkNumber MaxAcceleration;
    LoggedNetworkNumber MaxVelocity;
    LoggedNetworkNumber UpperKG;
    LoggedNetworkNumber kGWithCoral;

    public LiftPIDTuning()
    {
        kP = new LoggedNetworkNumber("Lift/PID/kP", 0.0);
        kI = new LoggedNetworkNumber("Lift/PID/kI", 0.0);
        kD = new LoggedNetworkNumber("Lift/PID/kD", 0.0);
        kV = new LoggedNetworkNumber("Lift/Feedforward/kV", 0.0);
        kG = new LoggedNetworkNumber("Lift/Feedforward/kG", 0.0);
        kS = new LoggedNetworkNumber("Lift/Feedforward/kS", 0.0);
        MaxAcceleration = new LoggedNetworkNumber("Lift/Constraints/MaxAcceleration", 0.0);
        MaxVelocity = new LoggedNetworkNumber("Lift/Constraints/MaxVelocity", 0.0);
        UpperKG = new LoggedNetworkNumber("Lift/Feedforward/UpperKG", 0.0);
        kGWithCoral = new LoggedNetworkNumber("Lift/Feedforward/kGWithCoral", 0.0);
    }
}
