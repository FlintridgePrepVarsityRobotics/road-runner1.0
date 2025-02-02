package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Projects.HWMapBasic;

@TeleOp(name = "drive")
public class drive extends LinearOpMode {
    public HWMapBasic robot = new HWMapBasic();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        double speed = .8;
        int rightPosition = 0;
        int leftPosition = 0;
        int noU = -8000;
        int pos = 2;
        double rDiff = .5;
        double lDiff = .5;
        int[] positions;
        robot.fRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /// init motor pos and encoder



        waitForStart();
        boolean isSpinning = false;
        while (opModeIsActive()) {
            boolean aButtonHeld = false;
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x*1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double ry = gamepad1.right_stick_y;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double fLeftPower = (y + x + rx) / denominator;
            double bLeftPower = (y - x + rx) / denominator;
            double fRightPower = (y - x - rx) / denominator;
            double bRightPower = (y + x - rx) / denominator;

            robot.fLeftWheel.setPower(fLeftPower * speed);
            robot.bLeftWheel.setPower(bLeftPower * speed);
            robot.fRightWheel.setPower(fRightPower * speed);
            robot.bRightWheel.setPower(bRightPower * speed);



        }
    }


    int[] WaitTillTargetReached(int tolerance, boolean lock){
        int leftDifference = Math.abs(robot.leftLift.getTargetPosition() - robot.leftLift.getCurrentPosition());
        int rightDifference = Math.abs(robot.rightLift.getTargetPosition() - robot.rightLift.getCurrentPosition());
        int check=102930293;
        while(leftDifference > tolerance || rightDifference > tolerance)

        {

            leftDifference = Math.abs(robot.leftLift.getTargetPosition() - robot.leftLift.getCurrentPosition());
            rightDifference = Math.abs(robot.rightLift.getTargetPosition() - robot.rightLift.getCurrentPosition());

            robot.leftLift.setPower(0.8);
            robot.rightLift.setPower(0.8);
            if (check == robot.rightLift.getCurrentPosition() + robot.leftLift.getCurrentPosition()) {
                break;
            }
            else {
                check = robot.rightLift.getCurrentPosition() + robot.leftLift.getCurrentPosition();
            }
            sleep(1);
            int a = robot.rightLift.getCurrentPosition();
            int c = robot.leftLift.getCurrentPosition();
            telemetry.addLine("current position: " + a + "," + c);
            telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
            telemetry.update();

        }
        int a = robot.rightLift.getCurrentPosition();
        int c = robot.leftLift.getCurrentPosition();
        telemetry.addLine("current position: " + a + "," + c);
        telemetry.addLine("target position: " + robot.leftLift.getTargetPosition());
        telemetry.update();
        int[] positions = new int[] {a,c};


        if(!lock)
        {
            robot.leftLift.setPower(0);
            robot.rightLift.setPower(0);
        }
        return(positions);

    }
    private void cycle() {

    }
}
