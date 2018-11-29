package frc.robot;

class SwerveDrive{

    // fl = 1
    // fr = 2
    // bl = 3
    // br = 4

    SwerveSystem system1;
    SwerveSystem system2;
    SwerveSystem system3;
    SwerveSystem system4;
    float W;
    float L;

    public SwerveDrive(SwerveSystem sys1, SwerveSystem sys2, SwerveSystem sys3, SwerveSystem sys4, float width, float length){
        system1 = sys1;
        system1 = sys1;
        system1 = sys1;
        system1 = sys1;
        W = width;
        L = length;       
    }
}