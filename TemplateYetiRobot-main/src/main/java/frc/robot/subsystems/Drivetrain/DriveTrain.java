package frc.robot.subsystems.Drivetrain;

//BELLOW CLASS COMMENTED OUT IS FOR GYRO I DIDN'T INSTALL LIBRARY AND IM TO LAZY TO :PPPPPPP
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class DriveTrain {
    public SwerveModule LeftBackMotor;
    public SwerveModule RightBackMotor;
    public SwerveModule LeftFrontMotor;
    public SwerveModule RightFrontMotor;

    //private final AHRS gyro = new AHRS(Port.kUSB); // DIDN'T INSTAL SO GIVING ERROR OR SOMETHING IM TO LAZY TO FIX
    private final Rotation2d flipGyro = new Rotation2d(Math.PI);

    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONTRAINTS = //
            new TrapezoidProfile.Constraints(
                    Constants.MAXVELOCITY, Constants.MAXANGULAR);

    private final PIDController yController = new PIDController(Constants.XCONTROLLERP, 0.0, 0.0);
    private final PIDController xController = new PIDController(Constants.YCONTROLLERP, 0.0, 0.0);
    //private final ProfiledPIDController thetaController = new ProfiledPIDController(Constants.THETACONTROLLERP, 0.0, 0.0, );
    //private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, new Rotation2d(0));
    private boolean isSwerveLock;
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private SwerveModule[] theGang = new SwerveModule[4];
    public DriveTrain(){
        LeftFrontMotor = new SwerveModule(1,1,1,false, 0);
        theGang[0] = LeftFrontMotor;
        RightFrontMotor = new SwerveModule(1,1,1,false, 0);
        theGang[1] = RightFrontMotor;
        LeftBackMotor = new SwerveModule(1,1,1,false, 0);
        theGang[2] = LeftBackMotor;
        RightBackMotor = new SwerveModule(1,1,1,false, 0);
        theGang[3] = RightBackMotor;

        //thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void drive(double forward, double side, double rotation, double p){
        for(int i = 0; i< 4; i++){
            theGang[i].vroom(forward, side, rotation, p);
        }
    }



//    public void zeroGyroscope() {
//        gyro.reset(); //GYRO EXSISTS IN A LIBRARY I DIDN'T INSTALL
//    }
//
//    public Rotation2d getGyroscopeRotation() {
//        // return gyro.getRotation2d();
//        return Rotation2d.fromDegrees(-gyro.getYaw()); //GYRO EXSISTS IN A LIBRARY I DIDN'T INSTALL
//    }

}
