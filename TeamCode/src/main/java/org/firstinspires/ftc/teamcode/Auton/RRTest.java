package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;
//package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "RRTest")
public class RRTest extends LinearOpMode {
    // lift class
    public class LeftLift {
        private DcMotorEx leftLift;

        public LeftLift(HardwareMap hardwareMap) {
            leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLift.setDirection(DcMotor.Direction.FORWARD);

        }
    }
    public class RightLift {
        private DcMotorEx rightLift;

        public RightLift(HardwareMap hardwareMap) {
            rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setDirection(DcMotor.Direction.REVERSE);

        }
    }

    // claw class
    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "Claw");
        }
    }
    public class LArm {
        private Servo lArm;

        public LArm(HardwareMap hardwareMap) {
            lArm = hardwareMap.get(Servo.class, "lArm");
        }
    }
    public class RArm {
        private Servo rArm;

        public RArm(HardwareMap hardwareMap) {
            rArm = hardwareMap.get(Servo.class, "rArm");
        }
    }
    public class RDiff {
        private Servo rDiff;

        public RDiff(HardwareMap hardwareMap) {
            rDiff = hardwareMap.get(Servo.class, "rDiff");
        }
    }
    public class LDiff {
        private Servo lDiff;

        public LDiff(HardwareMap hardwareMap) {
            lDiff = hardwareMap.get(Servo.class, "lDiff");
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
       // robot.init(hardwareMap);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        while (!isStarted()) {


        }
        waitForStart(); //wait for play button to be pressed
        // autonomous happens here
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(20, 0), Math.PI / 2)
                .splineTo(new Vector2d(7, 0), Math.PI / 2)
                .splineTo(new Vector2d(27, -26), Math.PI / 2)
                .splineTo(new Vector2d(60, -26), Math.PI / 2)
                .splineTo(new Vector2d(60, -36), Math.PI / 2)
                .splineTo(new Vector2d(16, -36), Math.PI / 2)
                .splineTo(new Vector2d(60, -36), Math.PI / 2)
                .splineTo(new Vector2d(60, -46), Math.PI / 2)
                .splineTo(new Vector2d(20, -46), Math.PI / 2)
                .splineTo(new Vector2d(60, -46), Math.PI / 2)
                .splineTo(new Vector2d(60, -56), Math.PI / 2)
                .splineTo(new Vector2d(20, -56), Math.PI / 2)
                .splineTo(new Vector2d(32, -30), Math.PI / 2)
                .splineTo(new Vector2d(5, -30), Math.PI / 2)
                .splineTo(new Vector2d(20, 0), Math.PI / 2)
                .splineTo(new Vector2d(15, 40), Math.PI / 2)
                .lineToYSplineHeading(33, Math.toRadians(0));


        //while(robot.right.isBusy()|| robot.left.isBusy()) {

        //telemetry.addData("Status", robot.armMotor.getCurrentPosition());
        // telemetry.update();
        //}


    }
}
