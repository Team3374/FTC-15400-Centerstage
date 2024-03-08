package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.AutoLiftCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Holder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveBase;
import org.firstinspires.ftc.teamcode.util.Storage;

import java.util.function.DoubleSupplier;

@SuppressWarnings("unused")
@TeleOp
public class mainOpMode extends CommandOpMode {
    //* create all subsystems
    private Drive drive;
    private Intake intake;
    private Holder holder;
    private Arm arm;
    private Lift lift;
    private Shooter shooter;

    //* create gamepads
    private GamepadEx gamepadOne;
    private GamepadEx gamepadTwo;

    //* run once, init code + adds items to the scheduler
    @Override
    public void initialize() {
        //* initialize subsystems
        drive = new Drive(new DriveBase(hardwareMap), true);

        intake = new Intake(hardwareMap);
        holder = new Holder(hardwareMap);
        arm = new Arm(hardwareMap);
        lift = new Lift(hardwareMap);
        shooter = new Shooter(hardwareMap);

        gamepadOne = new GamepadEx(gamepad1);
        gamepadTwo = new GamepadEx(gamepad2);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //* set pose estimate based on starting location
        Pose2d poseEstimate;

        if (Storage.currentColor.equals("blue")) {
            poseEstimate = new Pose2d(
                    Storage.currentPose.getX(),
                    Storage.currentPose.getY(),
                    Math.toRadians(90));
        } else if (Storage.currentColor.equals("red")) {
            poseEstimate = new Pose2d(
                    Storage.currentPose.getX(),
                    Storage.currentPose.getY(),
                    Math.toRadians(-90));
        } else {
            poseEstimate = new Pose2d(
                    Storage.currentPose.getX(),
                    Storage.currentPose.getY(),
                    Math.toRadians(0));
        }

        drive.setPoseEstimate(poseEstimate);

        //* assign drive commands
        drive.setDefaultCommand(
                new RunCommand(() -> {
                    drive.driveWithLimits(gamepadOne.getLeftY(), gamepadOne.getLeftX(), gamepadOne.getRightX());
                    drive.update();
                })
        );

        //* pose reset button, removes speed limiter
        gamepadTwo.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> {
                    drive.setPoseEstimate(new Pose2d(0, 0, 0));
                    Storage.currentColor = "none";
                })
        );

        //* intake commands
        gamepadOne.getGamepadButton(GamepadKeys.Button.A).whileHeld(
                new IntakeCommand(intake, holder, true)
        );

        gamepadOne.getGamepadButton(GamepadKeys.Button.B).whileHeld(
                new IntakeCommand(intake, holder, false)
        );

        //* arm commands
        gamepadOne.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ArmCommand(arm, 0.45)
        );

        gamepadOne.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ArmCommand(arm, 0.55)
        );

        //* lift commands
        schedule(
                new RunCommand(() -> {
                    double rightTriggerOne = gamepadOne.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                    double leftTriggerOne = gamepadOne.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                    double rightTriggerTwo = gamepadTwo.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                    double leftTriggerTwo = gamepadTwo.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

                    if (rightTriggerOne > 0.05 || leftTriggerOne > 0.05) {
                        DoubleSupplier input = () -> rightTriggerOne - leftTriggerOne;
                        new ManualLiftCommand(lift, arm, input, true);
                    } else if (rightTriggerTwo > 0.05 || leftTriggerTwo > 0.05) {
                        DoubleSupplier input = () -> rightTriggerTwo - leftTriggerTwo;
                        new ManualLiftCommand(lift, arm, input, false);
                    }
                })
        );

        gamepadOne.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new AutoLiftCommand(lift, arm, 1750)
        );

        gamepadOne.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new AutoLiftCommand(lift, arm, 0)
        );

        //* airplane shooter commands
        gamepadOne.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(shooter::up)
        );

        gamepadOne.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(shooter::down)
        );

        //* telemetry
        schedule(
                new InstantCommand(() -> {
                    telemetry.addData("x", drive.getPoseEstimate().getX());
                    telemetry.addData("y", drive.getPoseEstimate().getY());
                    telemetry.addData("heading", drive.getPoseEstimate().getHeading());
                    telemetry.addLine();
                    telemetry.addData("Lift Position", lift.getPosition());
                    telemetry.addLine();
                    telemetry.addData("Holder Sensor", arm.isDown());
                    telemetry.update();
                })
        );
    }
}
