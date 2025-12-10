#include <rclcpp/rclcpp.hpp>
#include <erocket/msg/flight_mode.hpp>
#include <erocket/constants.hpp>
#include <erocket/frame_transforms.h>

#include <erocket/vehicle_constants.hpp>
#include <erocket/controller/allocator.hpp>
#include <erocket/controller/state.hpp>
#include <erocket/controller/setpoint.hpp>
#include <erocket/controller/impls/attitude_pid.hpp>
#include <erocket/controller/impls/position_pid.hpp>

#include <chrono>

using namespace std::chrono;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;
using namespace erocket::msg;
using namespace erocket::constants;
using namespace erocket::constants::setpoint;
using namespace erocket::constants::controller;
using namespace erocket::constants::flight_mode;
using namespace erocket::frame_transforms;

/**
 * @brief Node that runs the controller for a 1-degree-of-freedom system
 */
class BaselinePIDController : public rclcpp::Node
{
public:
	explicit BaselinePIDController() : Node("baseline_pid_controller"),
    qos_profile_{rmw_qos_profile_sensor_data},
    qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},

    vehicle_constants_{std::make_shared<VehicleConstants>(this)},
    state_aggregator_{std::make_unique<StateAggregator>(this, qos_)},
    setpoint_aggregator_{std::make_unique<SetpointAggregator>(this, qos_)},
    attitude_controller_{this, qos_, state_aggregator_, setpoint_aggregator_, vehicle_constants_},
    position_controller_{this, qos_, vehicle_constants_, state_aggregator_, setpoint_aggregator_},
    allocator_{std::make_unique<Allocator>(this, qos_, vehicle_constants_)},

    flight_mode_{FlightMode::INIT},
    flight_mode_get_subscriber_{this->create_subscription<FlightMode>(
        FLIGHT_MODE_GET_TOPIC, qos_, 
        [this](const FlightMode::SharedPtr msg) {
            flight_mode_ = msg->flight_mode;
        }
    )}
    {
        if (position_controller_.is_controller_active() && !attitude_controller_.are_all_controllers_active()) {
            RCLCPP_ERROR(this->get_logger(), "Position controller requires all attitude controllers to be active.");
            throw std::runtime_error("Position controller requires all attitude controllers to be active.");
        }

        // TODO: change this for future
        controller_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.02), 
            std::bind(&BaselinePIDController::controller_callback, this)
        );
	}

private:
    rmw_qos_profile_t qos_profile_;
    rclcpp::QoS qos_;

    std::shared_ptr<VehicleConstants> vehicle_constants_;
    std::shared_ptr<StateAggregator> state_aggregator_;
    std::shared_ptr<SetpointAggregator> setpoint_aggregator_;
    AttitudePIDController attitude_controller_;
    PositionPIDController position_controller_;
    std::unique_ptr<Allocator> allocator_;

    //!< Time variables
    rclcpp::TimerBase::SharedPtr controller_timer_;

	//!< Auxiliary functions
    void controller_callback();

    std::atomic<uint8_t> flight_mode_;
    rclcpp::Subscription<FlightMode>::SharedPtr flight_mode_get_subscriber_;
};

/**
 * @brief Callback function for the controller
 */
void BaselinePIDController::controller_callback()
{
    if (flight_mode_ < FlightMode::ARM) {
        allocator_->compute_allocation_neutral();
        return;
    }

    else if (flight_mode_ == FlightMode::ARM) {
        static rclcpp::Time t0 = this->get_clock()->now();
        rclcpp::Time now = this->get_clock()->now();

        auto inner_servo_tilt_angle_radians = 0.0f;
        auto outer_servo_tilt_angle_radians = 0.0f;
        auto upwards_motor_thrust_pwm = NAN;
        auto downwards_motor_thrust_pwm = NAN;

        if (now - t0 < 3.0s) {
            auto max_servo_tilt_angle_radians = degrees_to_radians(vehicle_constants_->servo_max_tilt_angle_degrees_);
            inner_servo_tilt_angle_radians = sin(2.0 * M_PI * (now - t0).seconds()) * max_servo_tilt_angle_radians;
            outer_servo_tilt_angle_radians = sin(2.0 * M_PI * (now - t0).seconds() + M_PI_2) * max_servo_tilt_angle_radians;

        }

        if (now - t0 > 4.0s) {
            // Allocate motor thrust based on the computed torque
            upwards_motor_thrust_pwm = 0.0f;
            downwards_motor_thrust_pwm = 0.0f;
        }

        allocator_->indirect_actuation(
            inner_servo_tilt_angle_radians,
            outer_servo_tilt_angle_radians,
            upwards_motor_thrust_pwm,
            downwards_motor_thrust_pwm
        );
        return;
    }

    // only run controller when in mission
    else if (flight_mode_ == FlightMode::TAKE_OFF || flight_mode_ == FlightMode::IN_MISSION || flight_mode_ == FlightMode::LANDING) {
        double average_motor_thrust_newtons = allocator_->motor_thrust_curve_pwm_to_newtons(
            vehicle_constants_->default_motor_pwm_
        );

        if (position_controller_.is_controller_active()) {
            auto position_output = position_controller_.compute();
            setpoint_aggregator_->set_attitude_setpoint({position_output.desired_attitude});
            average_motor_thrust_newtons = position_output.u3;
        }

        auto allocator_input = attitude_controller_.compute(average_motor_thrust_newtons);

        allocator_->compute_allocation(allocator_input);
        return;
    }

    else if (flight_mode_ == FlightMode::ABORT) {
        allocator_->compute_allocation_neutral();

        rclcpp::shutdown();
        return;
    }
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard baseline pid controller node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BaselinePIDController>());

	rclcpp::shutdown();
	return 0;
}
