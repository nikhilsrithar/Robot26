package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import Team4450.Robot26.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.ctre.phoenix6.CANBus;

public class Hopper extends SubsystemBase {
    private final TalonFX hopperMotor = new TalonFX(Constants.HOPPER_MOTOR_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    private final TalonFX hopperMotor2 = new TalonFX(Constants.HOPPER_MOTOR_SECOND_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    private RobotContainer robotContainer;
    public Hopper() {
        // Configure motor neutral mode
        hopperMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration hopperCFG = new TalonFXConfiguration();

        // Neutral + inversion
        hopperCFG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hopperCFG.CurrentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.HOPPER_CURRENT_LIMIT);
        hopperCFG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        this.hopperMotor.getConfigurator().apply(hopperCFG);
    // Apply same configuration to the follower if it exists on the CAN bus
    this.hopperMotor2.getConfigurator().apply(hopperCFG);

        hopperMotor.set(0);
        hopperMotor2.setControl(new Follower(hopperMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void start() {
        hopperMotor.set(1);
        if (hopperMotor2.isConnected()) {
            hopperMotor2.setControl(new Follower(hopperMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    public void startWithScaling() {
        double percent = 1 * robotContainer.getVolatgePercent() * Constants.HOPPER_VOLTAGE_MULTIPLIER;
        hopperMotor.set(percent);
        if (hopperMotor2.isConnected()) {
            hopperMotor2.setControl(new Follower(hopperMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    public void startSlow() {
        hopperMotor.set(0.2);
        if (hopperMotor2.isConnected()) {
            hopperMotor2.setControl(new Follower(hopperMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    public void stop() {
        hopperMotor.set(0);
        if (hopperMotor2.isConnected()) {
            hopperMotor2.setControl(new Follower(hopperMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    public double getHooperCurrent() {
        return hopperMotor.getSupplyCurrent(true).getValueAsDouble();
    }
}
