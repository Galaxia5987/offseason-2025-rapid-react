package frc.robot.lib.universal_motor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import frc.robot.CURRENT_MODE
import frc.robot.Mode
import frc.robot.lib.extensions.kg2m
import frc.robot.lib.extensions.m

/**
 * Represents a universal wrapper for a motor, which abstracts the real and
 * simulated implementations.
 *
 * @param port The CAN ID of the motor controller.
 * @param canbus The CAN bus name (optional, default is the default bus).
 * @param config Configuration for the TalonFX motor controller.
 * @param momentOfInertia The moment of inertia used in simulation.
 * @param gearRatio The gear ratio between the motor and the mechanism (default
 * is 1.0).
 * @param linearSystemWheelDiameter The diameter of the wheel or spool for
 * linear mechanisms.
 * ```
 *        Do Not pass this parameter if the motor does not actuate a linear mechanism (e.g., elevator or a linear intake).
 * ```
 */
class UniversalTalonFX(
    port: Int,
    canbus: String = "",
    config: TalonFXConfiguration = TalonFXConfiguration(),
    momentOfInertia: MomentOfInertia = 0.003.kg2m,
    gearRatio: Double = 1.0,
    linearSystemWheelDiameter: Distance = 0.m,
) {
    private val motorIO: MotorIO =
        if (CURRENT_MODE == Mode.REAL)
            MotorIOReal(
                port,
                canbus,
                config,
                gearRatio,
                linearSystemWheelDiameter
            )
        else {
            MotorIOSim(
                momentOfInertia,
                config,
                gearRatio,
                linearSystemWheelDiameter
            )
        }
    val inputs: LoggedMotorInputs = motorIO.inputs

    /**
     * Sends a control request to the motor (e.g., voltage, velocity, position).
     *
     * @param control The control request to apply.
     */
    fun setControl(control: ControlRequest) = motorIO.setRequest(control)

    fun reset(angle: Angle) = motorIO.resetInternalEncoder(angle)

    fun updateInputs() = motorIO.updateInputs()
}
