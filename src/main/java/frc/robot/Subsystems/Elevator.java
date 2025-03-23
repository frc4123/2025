package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    
    private final TalonFX elevator = new TalonFX(Constants.CanIdCanivore.Elevator, "sim");
    private final DynamicMotionMagicTorqueCurrentFOC m_motionMagicCtrl = new DynamicMotionMagicTorqueCurrentFOC(Constants.Elevator.down, Constants.Elevator.velocity, Constants.Elevator.acceleration, Constants.Elevator.jerk);

    private final NetworkTable armStateTable = NetworkTableInstance.getDefault().getTable("Elevator State");
    private final DoublePublisher relativePublisher = armStateTable.getDoubleTopic("Elevator Relative Position: ").publish();

    private final TalonFXSimState elevatorSim = elevator.getSimState();

    // Simulated motion profile state
    private double simulatedPosition = 0.0;
    private double simulatedVelocity = 0.0;
    private double lastTargetPosition = 0.0;

    public Elevator() {
        configureMotor();
        SmartDashboard.putData("Elevator Mechanism", elevatorMech);
        simulatedPosition = elevator.getPosition().getValueAsDouble();
    }

    private void configureMotor() {
        var config = new TalonFXConfiguration();
        elevator.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs pidConfig = new Slot0Configs()
            .withKP(Constants.Elevator.kP)
            .withKI(Constants.Elevator.kI)
            .withKD(Constants.Elevator.kD)
            .withKV(Constants.Elevator.kV)
            .withKA(Constants.Elevator.kA)
            .withKG(Constants.Elevator.kG);   

        elevator.getConfigurator().apply(pidConfig);

        // Configure sensor direction here
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.Feedback.RotorToSensorRatio = 1.0;
        
        // Apply all configs
        elevator.getConfigurator().apply(config);
    }

    public void setPosition(double position) {
        elevator.setControl(m_motionMagicCtrl.withPosition(position));
        lastTargetPosition = position;
    }

    public double getRelativePosition() {
        return elevator.getPosition().getValueAsDouble();
    }

    public double getElevatorSwerveReduction(){
        double percentOfMaxHeight = getRelativePosition() / Constants.Elevator.up;
        double elevatorReductionPercent = Math.abs(percentOfMaxHeight - 1);
        return elevatorReductionPercent <= 0.1 ? 0.1 : elevatorReductionPercent;
    }

    // Mechanism2d visualization
    private final Mechanism2d elevatorMech = new Mechanism2d(1.0, 2.0);
    private final MechanismRoot2d elevatorRoot = elevatorMech.getRoot("ElevatorRoot", 0.5, 0.0);
    private final MechanismLigament2d elevatorLigament = elevatorRoot.append(
        new MechanismLigament2d("Elevator", 0.0, 90.0)
    );

    @Override
    public void periodic() {
        double elevatorHeightMeters = getRelativePosition() * Constants.Elevator.METERS_PER_MOTOR_ROTATION;
        elevatorLigament.setLength(elevatorHeightMeters);
        relativePublisher.set(getRelativePosition());
    }

    @Override
    public void simulationPeriodic() {
        // Configure sensor direction through motor config instead
        elevatorSim.setRawRotorPosition(simulatedPosition);
        elevatorSim.setRotorVelocity(simulatedVelocity);
        elevatorSim.setSupplyVoltage(12.0);
        
        final double dt = 0.020; // 20ms loop period
        
        double targetPosition = m_motionMagicCtrl.Position;
        double maxVelocity = Constants.Elevator.velocity; // rotations per second
        double maxAcceleration = Constants.Elevator.acceleration; // rotations per second squared

        double error = targetPosition - simulatedPosition;
        double direction = Math.signum(error);

        // Calculate stopping distance
        double stoppingDistance = (simulatedVelocity * simulatedVelocity) / (2 * maxAcceleration);

        if (Math.abs(error) <= stoppingDistance && error != 0) {
            // Decelerate
            double deceleration = -Math.signum(simulatedVelocity) * maxAcceleration;
            simulatedVelocity += deceleration * dt;
            
            // Prevent overshooting
            if (Math.signum(simulatedVelocity) != direction) {
                simulatedVelocity = 0;
                simulatedPosition = targetPosition;
            }
        } else {
            // Accelerate towards target
            double targetVelocity = maxVelocity * direction;
            double acceleration = (targetVelocity - simulatedVelocity) / dt;
            acceleration = Math.signum(acceleration) * Math.min(Math.abs(acceleration), maxAcceleration);
            simulatedVelocity += acceleration * dt;
        }

        // Apply velocity limits
        simulatedVelocity = Math.max(-maxVelocity, Math.min(simulatedVelocity, maxVelocity));

        // Update position
        simulatedPosition += simulatedVelocity * dt;

        // Snap to target if very close
        if (Math.abs(error) < 1e-6) {
            simulatedPosition = targetPosition;
            simulatedVelocity = 0.0;
        }

        // Update sim state
        elevatorSim.setRawRotorPosition(simulatedPosition);
        elevatorSim.setRotorVelocity(simulatedVelocity);
        
        // Add small delay to simulate CAN bus timing
        try { Thread.sleep(2); } catch (InterruptedException e) {}
    }
}