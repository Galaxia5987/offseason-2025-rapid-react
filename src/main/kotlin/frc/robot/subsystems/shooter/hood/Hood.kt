package frc.robot.subsystems.shooter.hood

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.UniversalTalonFX
import java.util.function.Supplier
import org.littletonrobotics.junction.Logger

class Hood : SubsystemBase(), SysIdable {

    private val motor =
        UniversalTalonFX(
            MOTOR_ID,
            config = MOTOR_CONFIG,
            gearRatio = MOTOR_TO_MECHANISM_RATIO
        )

    private val positionRequest = PositionTorqueCurrentFOC(0.0)
    private val voltageRequest = VoltageOut(0.0)

    private val encoder = CANcoder(ENCODER_ID)

    private var setpoint: Angle = 0.deg

    private val isAtSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, SETPOINT_TOLERANCE)
    }

    init {
        encoder.configurator.apply(ENCODER_CONFIG)
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun setAngle(angle: Angle): Command = runOnce {
        setpoint = angle
        motor.setControl(positionRequest.withPosition(angle))
    }

    fun setAngle(angle: Supplier<Angle>): Command = run {
        setpoint = angle.get()
        motor.setControl(positionRequest.withPosition(angle.get()))
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Subsystems/$name", motor.inputs)
        Logger.recordOutput("Subsystems/$name/isAtSetpoint", isAtSetpoint)
        Logger.recordOutput("Subsystems/$name/setpoint", setpoint)
    }
}
