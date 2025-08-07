package frc.robot.subsystems.roller

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.kg2m
import frc.robot.lib.extensions.kilogramSquareMeters
import frc.robot.lib.unified_canrange.UnifiedCANRange
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

@AutoLogOutput private var mechanism = LoggedMechanism2d(6.0, 4.0)
private var root = mechanism.getRoot("Wrist", 3.0, 2.0)
private val ligament =
    root.append(LoggedMechanismLigament2d("WristLigament", 0.25, 90.0))

class Roller : SubsystemBase() {
    private val motor =
        UniversalTalonFX(MOTOR_PORT, momentOfInertia = 0.002.kg2m)

    private val auxiliaryMotor =
        UniversalTalonFX(
            AUXILIARY_MOTOR_PORT,
            momentOfInertia = (0.002).kilogramSquareMeters,
        )

    private val voltageRequest = VoltageOut(0.0)

    private val rangeSensor =
        UnifiedCANRange(
            subsystemName = "$name",
            port = SENSOR_ID,
            canbus = "",
            configuration = CANrangeConfiguration()
        )

    @AutoLogOutput val hasBall = Trigger { rangeSensor.isInRange }

    private fun setVoltage(voltage: Voltage): Command = runOnce {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun intake(): Command = setVoltage(INTAKE)

    fun outtake(): Command = setVoltage(OUTTAKE)

    fun stop(): Command = setVoltage(STOP)

    override fun periodic() {
        motor.updateInputs()
        rangeSensor.updateInputs()
        Logger.processInputs("Subsystems/$name", motor.inputs)
    }
}
