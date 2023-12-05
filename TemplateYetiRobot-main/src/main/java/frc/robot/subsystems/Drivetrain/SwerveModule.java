package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
//import com.swervedrivespecialties.swervelib.SdsModuleConfigurations; //DIDN't DOWNLOAD THIS LIBRARY :PPPP
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;



//rlly good link
//https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_w_p_i___talon_f_x.html#a33ebd356cf623a9f5ef0634d7284105d

public class SwerveModule {
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX steerMotor;
    CANCoder absoluteEncoder;
//    private final PIDController drivePIDController =
//            new PIDController(
//                    Constants.DRIVEMOTORP,
//                    Constants.DRIVEMOTORI,
//                    Constants.DRIVEMOTORD,
//                    new TrapezoidProfile.Constraints((3 * Math.PI),  /*546 **/ Math.PI) //1080)
//            );

    public SwerveModule(int d, int s, int i, boolean a, double f){
        int driveID = d;
        int steerID = s;
        int encoderId = i;
        boolean absEncoderReversed = a;
        double absEncoderDegree = f;

        absoluteEncoder = new CANCoder(encoderId);
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        absoluteEncoder.configMagnetOffset(absEncoderDegree);
        absoluteEncoder.configSensorDirection(absEncoderReversed);
        absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
        absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 250);

        driveMotor = new WPI_TalonFX(driveID);
        steerMotor = new WPI_TalonFX(steerID);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1));
        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1));
        steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1));
        steerMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1));

        driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

        steerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        steerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

        //steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double DrivePosition() {
        return Math.toRadians(absoluteEncoder.getAbsolutePosition());
    }

    public void move(double p){
        driveMotor.set(p);
    }

    public void stop() {
        driveMotor.setVoltage(0.0);
        steerMotor.set(0.0);
    }

    public void vroom(double forward, double side, double rotation, double p) {
        ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(forward, side, Math.PI / 2.0, Rotation2d.fromDegrees(rotation));
        driveMotor.set(p);
    }

//    public void setDesiredState(SwerveModuleState desiredState) {
//        double driveVelocity = driveMotor.getSelectedSensorVelocity() * 10 / 2048
//                * SdsModuleConfigurations.MK4_L2.getDriveReduction() *
//                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;;
//        double steerAngle = getSteerPosition();
//
//        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01
//                && Math.abs(desiredState.angle.getRadians() - steerAngle) < 0.05) {
//            stop();
//            return;
//        }
//        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(steerAngle));
//
//        final double driveOutput =
//                drivePIDController.calculate(driveVelocity, desiredState.speedMetersPerSecond)
//                        + driveFeedforward.calculate(desiredState.speedMetersPerSecond);
//
//        final double steerOutput =
//                steeringPIDController.calculate(steerAngle, desiredState.angle.getRadians())
//                        + steerFeedforward.calculate(steeringPIDController.getSetpoint().velocity);
//
//        driveMotor.setVoltage(desiredState.speedMetersPerSecond / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
//                * DriveConstants.MAX_VOLTAGE);
//        // driveMotor.setVoltage(driveOutput);
//        // steerMotor.set(steeringPIDController.calculate(getSteerPosition(), desiredState.angle.getDegrees()));
//        // steerMotor.set(steeringPIDController.calculate(getSteerPosition(), 45));
//        steerMotor.setVoltage(steerOutput);
////
////        steerMotor.set(ControlMode.Position, desiredState.angle.getDegrees() * DriveConstants.DEGREES_TO_FALCON);
//    }

}
