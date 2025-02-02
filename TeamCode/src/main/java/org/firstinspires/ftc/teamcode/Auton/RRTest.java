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

        public class LeftLiftUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    leftLift.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = leftLift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > -6880.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    leftLift.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }
        public Action leftLiftUp() {
            return new LeftLiftUp();
        }

        public class LeftLiftMed implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    leftLift.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = leftLift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > -5300.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    leftLift.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action leftLiftMed() {
            return new LeftLiftMed();
        }

        public class LeftLiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftLift.setPower(-0.8);
                    initialized = true;
                }

                double pos = leftLift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < -100.0) {
                    return true;
                } else {
                    leftLift.setPower(0);
                    return false;
                }
            }
        }

        public Action leftLiftDown() {
            return new LeftLiftDown();
        }
    }


    public class RightLift {
        private DcMotorEx rightLift;

        public RightLift(HardwareMap hardwareMap) {
            rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setDirection(DcMotor.Direction.REVERSE);

        }

        public class RightLiftUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    rightLift.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = rightLift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > -6880.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    rightLift.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }
        public Action rightLiftUp() {
            return new RightLiftUp();
        }
        public class RightLiftMed implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    rightLift.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = rightLift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > -5300.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    rightLift.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action rightLiftMed() {
            return new RightLiftMed();
        }

        public class RightLiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightLift.setPower(-0.8);
                    initialized = true;
                }

                double pos = rightLift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < -100.0) {
                    return true;
                } else {
                    rightLift.setPower(0);
                    return false;
                }
            }
        }

        public Action rightLiftDown() {
            return new RightLiftDown();
        }
    }


    // claw class
    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "Claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class LArm {
        private Servo lArm;

        public LArm(HardwareMap hardwareMap) {
            lArm = hardwareMap.get(Servo.class, "lArm");
        }

        public class CloselArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lArm.setPosition(1);
                return false;
            }
        }

        public Action closelArm() {
            return new CloselArm();
        }

        public class OpenlArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lArm.setPosition(0);
                return false;
            }
        }

        public Action openlArm() {
            return new OpenlArm();
        }
    }

    public class RArm {
        private Servo rArm;

        public RArm(HardwareMap hardwareMap) {
            rArm = hardwareMap.get(Servo.class, "rArm");
        }

        public class CloserArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rArm.setPosition(0);
                return false;
            }
        }

        public Action closerArm() {
            return new CloserArm();
        }

        public class OpenrArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rArm.setPosition(1);
                return false;
            }
        }

        public Action openrArm() {
            return new OpenrArm();
        }
    }

    public class RDiff {
        private Servo rDiff;

        public RDiff(HardwareMap hardwareMap) {
            rDiff = hardwareMap.get(Servo.class, "rDiff");
        }

        public class CloserDiff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rDiff.setPosition(.25);
                return false;
            }
        }

        public Action closerDiff() {
            return new CloserDiff();
        }

        public class OpenrDiff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rDiff.setPosition(.5);
                return false;
            }
        }

        public Action openrDiff() {
            return new OpenrDiff();
        }
    }

    public class LDiff {
        private Servo lDiff;

        public LDiff(HardwareMap hardwareMap) {
            lDiff = hardwareMap.get(Servo.class, "lDiff");
        }

        public class CloselDiff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lDiff.setPosition(.75);
                return false;
            }
        }

        public Action closelDiff() {
            return new CloselDiff();
        }

        public class OpenlDiff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lDiff.setPosition(.5);
                return false;
            }
        }

        public Action openlDiff() {
            return new OpenlDiff();
        }
    }

    public class LBar {
        private Servo lBar;

        public LBar(HardwareMap hardwareMap) {
            lBar = hardwareMap.get(Servo.class, "lLink");
        }

        public class CloselBar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lBar.setPosition(0);
                return false;
            }
        }

        public Action closelBar() {
            return new CloselBar();
        }


    }

    public class RBar {
        private Servo rBar;

        public RBar(HardwareMap hardwareMap) {
            rBar = hardwareMap.get(Servo.class, "rLink");
        }

        public class CloserBar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rBar.setPosition(0);
                return false;
            }
        }

        public Action closerBar() {
            return new CloserBar();
        }


    }


    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        // robot.init(hardwareMap);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        LeftLift leftLift = new LeftLift(hardwareMap);
        RightLift rightLift = new RightLift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        LArm lArm = new LArm(hardwareMap);
        RArm rArm = new RArm(hardwareMap);
        RDiff rDiff = new RDiff(hardwareMap);
        LDiff lDiff = new LDiff(hardwareMap);
        RBar rBar = new RBar(hardwareMap);
        LBar lBar = new LBar(hardwareMap);
        int visionOutputPosition = 1;


        while (!isStarted()) {


        }
        waitForStart(); //wait for play button to be pressed
        // autonomous happens here
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)

                .splineToConstantHeading(new Vector2d(27.6, 0), Math.PI / 2);
        Action trajectoryActionCloseOut = tab1.endTrajectory()
                .build();

//        TrajectoryActionBuilder tab15 = drive.actionBuilder(new Pose2d(22, 0, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(27.6, 0), Math.PI / 2);
//        Action trajectoryActionCloseOut15 = tab15.endTrajectory()
//                .build();
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(27.6, 0, Math.toRadians(0)))

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
                .splineToConstantHeading(new Vector2d(32, -30), Math.PI / 2);
        Action trajectoryActionCloseOut2 = tab2.endTrajectory()
                .build();

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(32, -30, Math.toRadians(0)))

                .splineToConstantHeading(new Vector2d(5, -30), 0);
        Action trajectoryActionCloseOut3 = tab3.endTrajectory()
                .build();

        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(5, -30, Math.toRadians(0)))

                .splineToConstantHeading(new Vector2d(27.6, 0), 0);
        Action trajectoryActionCloseOut4 = tab4.endTrajectory()
                .build();

        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(20, 0, Math.toRadians(0)))

                .splineToConstantHeading(new Vector2d(15, -40), Math.PI / 2);
        Action trajectoryActionCloseOut5 = tab5.endTrajectory()
                .build();


        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(lArm.closelArm());
        Actions.runBlocking(rArm.closerArm());
        Actions.runBlocking(lDiff.closelDiff());
        Actions.runBlocking(rDiff.closerDiff());
        Actions.runBlocking(rBar.closerBar());
        Actions.runBlocking(lBar.closelBar());

        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),
                        leftLift.leftLiftMed(),
                        rightLift.rightLiftMed(),
                        trajectoryActionCloseOut,
                        leftLift.leftLiftUp(),
                        rightLift.rightLiftUp(),
                        claw.openClaw(),
                        tab2.build(),
                        leftLift.leftLiftDown(),
                        rightLift.rightLiftDown(),
                        trajectoryActionCloseOut2,
                        rArm.openrArm(),
                        lArm.openlArm(),
                        rDiff.openrDiff(),
                        lDiff.openlDiff(),
                        tab3.build(),
                        trajectoryActionCloseOut3,
                        claw.closeClaw(),
                        rArm.closerArm(),
                        lArm.closelArm(),
                        rDiff.closerDiff(),
                        lDiff.closelDiff(),
                        tab4.build(),
                        leftLift.leftLiftMed(),
                        rightLift.rightLiftMed(),
                        trajectoryActionCloseOut4,
                        leftLift.leftLiftUp(),
                        rightLift.rightLiftUp(),
                        claw.openClaw(),

                        tab5.build(),
                        leftLift.leftLiftDown(),
                        rightLift.rightLiftDown(),
                        trajectoryActionCloseOut5


                )
        );
        //while(robot.right.isBusy()|| robot.left.isBusy()) {

        //telemetry.addData("Status", robot.armMotor.getCurrentPosition());
        // telemetry.update();
        //}


    }
}



