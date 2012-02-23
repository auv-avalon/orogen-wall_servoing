/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WALL_SERVOING_SINGLESONARSERVOING_TASK_HPP
#define WALL_SERVOING_SINGLESONARSERVOING_TASK_HPP

#include "wall_servoing/SingleSonarServoingBase.hpp"
#include <sonar_detectors/CenterWallEstimation.hpp>
#include <sonar_detectors/MWallEstimation.hpp>
#include <sonar_detectors/SonarWallMap.hpp>

namespace wall_servoing {
    
    enum WallState
    {
        NO_WALL_FOUND = 0,
        WALL_TO_NEAR,
        DISTANCE_DIFF,
        ANGLE_DIFF,
        WALL_FOUND
    };
    
    class SingleSonarServoing : public SingleSonarServoingBase
    {
	friend class SingleSonarServoingBase;
    protected:
        sonar_detectors::CenterWallEstimation* centerWallEstimation;
        sonar_detectors::MWallEstimation* mWallEstimation;
        base::samples::RigidBodyState current_orientation;
        sonar_detectors::SonarWallMap wall_map;
        States last_state;
        WallState wall_state;
        bool do_wall_servoing;
        int checking_count;
        int exploration_checking_count;
        int detected_corner_count;
        const static int checking_wall_samples = 50;
        const static int exploration_mode_samples = 100;
        double last_distance_to_wall;
        base::Angle last_angle_to_wall;
        base::Angle current_wall_angle;
        const static double check_distance_threshold = 1.0;
        const static double check_angle_threshold = 0.25 * M_PI;
        double origin_wall_angle;
        double wall_servoing_direction;

    public:
        SingleSonarServoing(std::string const& name = "wall_servoing::SingleSonarServoing", TaskCore::TaskState initial_state = Stopped);

	~SingleSonarServoing();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
         */
        // bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recovered() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

