package frc.robot.subsystems.Lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.s_LIFT_CONSTANTS.*;


public class PhysicalLiftIO implements ILiftIO {
    POMSparkMax motor;
    RelativeEncoder encoder;
    private ProfiledPIDController pidController;
    private ElevatorFeedforward feedforward;
    private POMDigitalInput foldSwitch;
    private POMDigitalInput brakeSwitch;
    private LiftPIDTuning pidConstants;
    private BooleanSupplier isCoralIn;

    public PhysicalLiftIO(BooleanSupplier isCoralIn) {
        motor = new POMSparkMax(ELEVATOR_ID);
        feedforward = new ElevatorFeedforward(KS, KG, KV);
        pidController = new ProfiledPIDController(KP, KI, KD,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        // TODO: Remove these bloody comments, they're bloody annoying.
        // feedforward = new ElevatorFeedforward( pidConstants.getKs(),
        // pidConstants.getKg(), pidConstants.getKv());
        // pidController = new ProfiledPIDController(pidConstants.getKp(),
        // pidConstants.getKi(), pidConstants.getKd(), new
        // TrapezoidProfile.Constraints(pidConstants.getMaxVelocity(),pidConstants.getMaxAcceleration()));

        pidConstants = new LiftPIDTuning();

        encoder = motor.getEncoder();
        this.isCoralIn = isCoralIn;

        foldSwitch = new POMDigitalInput(FOLD_SWITCH);
        brakeSwitch = new POMDigitalInput(BRAKE_SWITCH);
        pidController.setTolerance(TOLERANCE);

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(SparkBaseConfig.IdleMode.kCoast).inverted(true) // assuming this inverts the motor, if it doesn't, I'm cooked
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION);

        // config.softLimit.forwardSoftLimit(FORWARD_SOFT_LIMIT);
        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        encoder.setPosition(0);

    }

    @Override
    public void updateInputs(LiftIOInputs inputs) {
        inputs.motorConnected = true /* turnConnectedDebouncer.calculate(sparkStickyFault) */;
        inputs.elevatorVelocity = encoder.getVelocity();
        inputs.elevatorPosition = encoder.getPosition();
        inputs.elevatorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        // Voltage
        inputs.foldSwitch = foldSwitch.get();
        setPidValues();
        resetIfPressed();
    }

    private void resetEncoder() {
        encoder.setPosition(0);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setVoltageWithCoral(double voltage) {
        motor.setVoltage(voltage + KG_OF_CORAL);
    }

    @Override
    public void setGoal(double goal) {
        pidController.setGoal(goal);
        setVoltage(getFeedForwardVelocity(pidController.getSetpoint().velocity)
                + pidController.calculate(encoder.getPosition()));
    }

    @Override
    public BooleanSupplier atGoal() {
        return () -> pidController.atGoal();
    }

    @Override
    public void stopMotor() {
        setVoltage(0 + getFeedForwardVelocity(0));
    }

    @Override
    public void resistGravity() {
        setVoltage(getFeedForwardVelocity(0));
    } // TODO: FREEDOM FOR SCOTLAND!!!!!

    boolean lastSwitchState = false;

    // @Override
    // public void resetlfPressed() {
    // if (DriverStation.isEnabled()) {
    // if (foldSwitch.get()) {
    // encoder.setPosition(0);
    // if (!lastSwitchState) {
    // motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake),
    // ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters);
    // }
    // lastSwitchState = true;
    // } else if (lastSwitchState) {
    // motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast),
    // ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters);
    // lastSwitchState = false;
    // }
    // } else {
    // motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake),
    // ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters);
    // }

    // }

    boolean isBrake = true;

    @Override
    public void resetIfPressed() {
        if (foldSwitch.get()) {
            resetEncoder();
        }
        if (DriverStation.isEnabled()) {
            if (foldSwitch.get()) {
                if (!isBrake) {
                    motor.configure(new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake), SparkBase.ResetMode.kNoResetSafeParameters,
                            SparkBase.PersistMode.kNoPersistParameters);
                    isBrake = true;
                }
            } else {
                if (isBrake) {
                    motor.configure(new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kCoast), SparkBase.ResetMode.kNoResetSafeParameters,
                            SparkBase.PersistMode.kNoPersistParameters);
                    isBrake = false;
                }
            }
        } else {
            if (brakeSwitch.get()) {
                if (isBrake) {
                    motor.configure(new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kCoast),
                            SparkBase.ResetMode.kNoResetSafeParameters,
                            SparkBase.PersistMode.kNoPersistParameters);
                    isBrake = false;
                }
                resetEncoder();
            } else {
                if (!isBrake) {
                    motor.configure(new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake),
                            SparkBase.ResetMode.kNoResetSafeParameters,
                            SparkBase.PersistMode.kNoPersistParameters);
                    isBrake = true;
                }
            }
        }
    }

    @Override
    public void setPidValues() {
        pidController.setP(pidConstants.getkP());
        pidController.setI(pidConstants.getkI());
        pidController.setD(pidConstants.getkD());
        pidController.setConstraints(
                new TrapezoidProfile.Constraints(pidConstants.getMaxVelocity(), pidConstants.getMaxAcceleration()));
        feedforward = new ElevatorFeedforward(pidConstants.getkS(), pidConstants.getkG(), pidConstants.getkV());

    }

    private double getFeedForwardVelocity(double velocity) {
        org.littletonrobotics.junction.Logger.recordOutput("real kG", feedforward.getKg());
        double voltage = feedforward.calculate(velocity);
        if (isCoralIn.getAsBoolean()) {
            voltage += pidConstants.getkGWithCoral();
        }

        if (encoder.getPosition() >= UPPER_POSITION) {
            voltage += pidConstants.getUpperKG();
        }
        return voltage;
    }

    @Override
    public void setFeedForward(double velocity) {
        motor.setVoltage(getFeedForwardVelocity(velocity));
    }

    @Override
    public boolean isPressed() {
        return foldSwitch.get();
    }

    @Override
    public void setVoltageWithResistGravity(double voltage) {
        motor.setVoltage(getFeedForwardVelocity(0) + voltage);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void resetPID() {
        pidController.reset(encoder.getPosition(), encoder.getVelocity());
    }

    @Override
    public void resetPID(double newGoal) {
        if (newGoal - encoder.getPosition() > 0) {
            pidController.reset(encoder.getPosition(), Math.max(encoder.getVelocity(), getFeedForwardVelocity(1)));
        } else {
            pidController.reset(encoder.getPosition(), Math.min(encoder.getVelocity(), getFeedForwardVelocity(1)));
        }
    }

}
