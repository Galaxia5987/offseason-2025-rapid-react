package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rps
import frc.robot.lib.extensions.sec

val SLOW_ROTATION = 10.rps
const val MAIN_MOTOR_PORT = 5
const val AUX_MOTOR_PORT = 6
val STATOR_CURRENT_LIMIT = 100.amps
val SUPPLY_CURRENT_LIMIT = 50.amps
val GAINS = Gains(kP = 2.0)
val TOLERANCE = 0.1.rps
val AT_SET_VELOCITY_DEBOUNCE = 0.2.sec
val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        Feedback = FeedbackConfigs().apply { RotorToSensorRatio = 1.0 }
        Slot0 = GAINS.toSlotConfig()

        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = STATOR_CURRENT_LIMIT[amps]
                SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT[amps]
            }
    }
