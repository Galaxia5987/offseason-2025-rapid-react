package frc.robot.robotstate

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Commands.parallel
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import frc.robot.drive
import frc.robot.flywheel
import frc.robot.hopper
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rotationFromPoint
import frc.robot.lib.extensions.rps
import frc.robot.lib.getPose2d
import frc.robot.roller
import frc.robot.subsystems.drive.alignToPose
import frc.robot.subsystems.shooter.flywheel.SLOW_ROTATION
import frc.robot.subsystems.shooter.hood.HoodAngles

val robotDistanceFromBasket
    get() = drive.pose.distanceFromPoint(HUB_LOCATION.translation)
val turretRotationToBasket: Angle
    get() {
        val robotHeading: Angle = drive.pose.rotation.degrees.deg
        val angleToBasket =
            drive.pose.rotationFromPoint(HUB_LOCATION.translation)
        var relativeAngle = angleToBasket - robotHeading
        val maxAngle = 135.deg
        relativeAngle =
            when {
                relativeAngle > maxAngle -> maxAngle
                relativeAngle < -maxAngle -> -maxAngle
                else -> relativeAngle
            }

        return relativeAngle
    }

val hoodAngle
    get() =
        when {
            robotDistanceFromBasket in HoodAngles.NEAR.range ->
                HoodAngles.NEAR.angle
            robotDistanceFromBasket in HoodAngles.MID.range ->
                HoodAngles.MID.angle
            robotDistanceFromBasket in HoodAngles.FAR.range ->
                HoodAngles.FAR.angle
            else -> 45.deg
        }

val flywheelTargetVelocity
    get() =
        when (robotDistanceFromBasket[m]) {
            in 0.2..0.6 -> 10
            in 0.6..1.0 -> 20
            in 1.0..1.4 -> 30
            in 1.4..1.8 -> 40
            in 1.8..2.2 -> 50
            in 2.2..2.6 -> 60
            in 2.6..3.0 -> 70
            in 3.0..3.4 -> 80
            in 3.4..3.8 -> 90
            in 3.8..4.0 -> 100
            else -> SLOW_ROTATION[rps]
        }.rps

fun driveToShootingPoint() =
    drive
        .defer {
            val robotTranslation = drive.pose.translation
            val setpoint =
                if (
                    INNER_SHOOTING_AREA.getDistance(robotTranslation) >
                        OUTER_SHOOTING_AREA.getDistance(robotTranslation)
                ) {
                    INNER_SHOOTING_AREA.nearest(robotTranslation)
                } else {
                    OUTER_SHOOTING_AREA.nearest(robotTranslation)
                }
            alignToPose((getPose2d(setpoint)))
        }
        .withName("Drive/Drive to shooting point")

fun shooting() =
    sequence(
            drive.lock(),
            waitUntil(flywheel.isAtSetVelocity),
            hopper.start(),
            roller.intake()
        )
        .withName("$COMMAND_NAME_PREFIX/Shooting")

fun stopShooting() =
    parallel(hopper.stop(), roller.stop())
        .withName("$COMMAND_NAME_PREFIX/StopShooting")

fun intaking() =
    parallel(roller.intake(), hopper.start())
        .withName("$COMMAND_NAME_PREFIX/Intake")

fun stopIntaking() =
    parallel(roller.stop(), hopper.stop())
        .withName("$COMMAND_NAME_PREFIX/StopIntake")
