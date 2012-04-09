/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WALL_SERVOING_DUALSONARSERVOING_TASK_HPP
#define WALL_SERVOING_DUALSONARSERVOING_TASK_HPP

#include "wall_servoing/DualSonarServoingBase.hpp"

namespace wall_servoing {
    
    struct ObstacleDistance
    {
        double last_distance;
        base::Time time_last_distance;
        std::vector<double> distance_collector;
        bool dirty;
        void checkForTimeout(double seconds);
        void addNewDistanceSample(double distance);
        ObstacleDistance() : last_distance(-1), dirty(false) {}
    };
    
    class DualSonarServoing : public DualSonarServoingBase
    {
	friend class DualSonarServoingBase;
    protected:
        double front_sonar_front_left_angle;
        double front_sonar_front_right_angle;
        double front_sonar_right_left_angle;
        double front_sonar_right_right_angle;
        double rear_sonar_left_angle;
        double rear_sonar_right_angle;
        
        base::samples::RigidBodyState current_orientation;
        ObstacleDistance wall_front_dist;
        ObstacleDistance wall_right_front_dist;
        ObstacleDistance wall_right_rear_dist;
        double last_target_heading;
        States last_state;
        bool handle_corner;
        
        double sonar_span_x;
        double sonar_span_y;
        double distance_sample_timeout;
        
        double no_sonar_features_timeout;
        base::Time last_valid_feature_front_front;
        base::Time last_valid_feature_front_right;
        base::Time last_valid_feature_rear_right;
        
    protected:
        bool isInRange(const double& angle, const double& left_limit, const double& right_limit) const;
        void receiveDistanceFromFeature(const base::samples::LaserScan& feature, double& distance) const;
        
    public:
        DualSonarServoing(std::string const& name = "wall_servoing::DualSonarServoing", TaskCore::TaskState initial_state = Stopped);
        DualSonarServoing(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

	~DualSonarServoing();

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
         * Call recover() to go back in the Runtime state.
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

