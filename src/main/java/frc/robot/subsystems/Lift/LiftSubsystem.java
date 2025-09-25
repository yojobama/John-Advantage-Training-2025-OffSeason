package frc.robot.subsystems.Lift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LiftSubsystem extends SubsystemBase {
    ILiftIO m_LiftIO;
    LiftIOInputsAutoLogged m_LiftInputs;

    public LiftSubsystem(ILiftIO liftIO) {
        m_LiftIO = liftIO;
        m_LiftInputs = new LiftIOInputsAutoLogged();
    }

    public void periodic() {
        m_LiftIO.updateInputs(m_LiftInputs);
        Logger.processInputs("Lift/Lift", m_LiftInputs);
    }

    public ILiftIO getIO() {
        return m_LiftIO;
    }
}
