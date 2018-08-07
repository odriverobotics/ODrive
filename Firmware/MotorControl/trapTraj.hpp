class TrapezoidalTrajectory {
private:
        float yAccel_;

        float Xi_;
        float Vi_;
        float Ai_;

        float Xf_;
        float Vf_;
        float Af_;

        float Ar_;
        float Dr_;
        float Vr_;

        float Ta_;
        float Tv_;
        float Td_;
        float Tav_;

public:
    struct TrajectoryStep_t{
        float Y;
        float Yd;
        float Ydd;
    };

    TrapezoidalTrajectory();

    float planTrapezoidal(float Xf, float Xi,
                    float Vf, float Vi,
                    float Af, float Ai,
                    float Vmax, float Amax, float Dmax
                    );
    
    TrajectoryStep_t evalTrapTraj(float t);

    ~TrapezoidalTrajectory();
}