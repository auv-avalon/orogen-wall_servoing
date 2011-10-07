/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WALL_SERVOING_TASK_TASK_HPP
#define WALL_SERVOING_TASK_TASK_HPP

#include "wall_servoing/TaskBase.hpp"
#include <sonar_detectors/SonarBeamProcessing.hpp>
#include <sonar_detectors/WallEstimation.hpp>
#include <sonar_detectors/DistanceEstimation.hpp>

namespace wall_servoing {
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        sonar_detectors::SonarBeamProcessing* processing;
        sonar_detectors::WallEstimation* wallEstimation;
        sonar_detectors::DistanceEstimation* distanceEstimation;
        States last_state;
        int checking_count;
        const static int checking_wall_samples = 50;
        double last_distance_to_wall;
        double last_angle_to_wall;
        const static double check_distance_threshold = 0.5;
        const static double check_angle_threshold = 0.2;
        double origin_wall_angle;
        double current_wall_angle;
        bool wall_checking_done;

    public:
        Task(std::string const& name = "sonarvizkit::Task", TaskCore::TaskState initial_state = Stopped);

	~Task();

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

