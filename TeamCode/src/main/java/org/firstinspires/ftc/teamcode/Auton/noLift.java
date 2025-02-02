package org.firstinspires.ftc.teamcode.Auton;
//package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@Autonomous(name = "noLift")
public class noLift extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        while (!isStarted()) {


        }
        waitForStart(); //wait for play button to be pressed
        // autonomous happens here
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(20, 0), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(27, 0), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(27, -29), 0)
                .splineToConstantHeading(new Vector2d(56, -29), 0)
                .splineToConstantHeading(new Vector2d(56, -36), 0)
                .splineToConstantHeading(new Vector2d(15, -36), 0)
                .splineToConstantHeading(new Vector2d(56, -36), 0)
                .splineToConstantHeading(new Vector2d(56, -45), 0)
                .splineToConstantHeading(new Vector2d(15, -45), 0)
                .splineToConstantHeading(new Vector2d(56, -45), 0)
                .splineToConstantHeading(new Vector2d(56, -54), 0)
                .splineToConstantHeading(new Vector2d(15, -54), 0)
                .splineToConstantHeading(new Vector2d(32, -30), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(5, -30), 0)
                .splineToConstantHeading(new Vector2d(20, 0), 0)
                .splineToConstantHeading(new Vector2d(15, -40), Math.PI / 2);
        Action trajectoryActionCloseOut = tab1.endTrajectory()
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),

                        trajectoryActionCloseOut
                )
        );
    }
}
