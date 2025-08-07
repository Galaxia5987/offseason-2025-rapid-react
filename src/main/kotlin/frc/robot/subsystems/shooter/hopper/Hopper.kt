package frc.robot.subsystems.shooter.hopper

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.volts
import frc.robot.lib.unified_canrange.UnifiedCANRange
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Hopper : SubsystemBase() {

    private val motor: UniversalTalonFX =
        UniversalTalonFX(MOTOR_ID, config = MOTOR_CONFIG)
    private val distanceSensor =
        UnifiedCANRange(
            DISTANCE_SENSOR_ID,
            configuration = DISTANCE_SENSOR_CONFIG,
            subsystemName = name
        )
    private val voltageRequest = VoltageOut(0.0)

    val hasBall = Trigger { distanceSensor.isInRange }

    private fun setVoltage(voltage: Voltage): Command = runOnce {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun start(): Command = setVoltage(INTAKE_VOLTAGE)

    fun stop(): Command = setVoltage(0.volts)

    override fun periodic() {
        motor.updateInputs()
        distanceSensor.updateInputs()
        Logger.processInputs("Subsystems/$name", motor.inputs)
    }
}
