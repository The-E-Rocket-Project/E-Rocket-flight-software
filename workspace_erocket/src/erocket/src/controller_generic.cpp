#include <rclcpp/rclcpp.hpp>
#include <erocket/msg/flight_mode.hpp>
#include <erocket/constants.hpp>
#include <erocket/frame_transforms.h>

#include <erocket/vehicle_constants.hpp>
#include <erocket/controller/allocator.hpp>
#include <erocket/controller/state.hpp>
#include <erocket/controller/setpoint.hpp>
#include <erocket/controller/impls/generic_controller.hpp>

#include <chrono>

using namespace std::chrono;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;
using namespace erocket::msg;
using namespace erocket::constants;
using namespace erocket::constants::setpoint;
using namespace erocket::constants::controller_generic;
using namespace erocket::constants::flight_mode;
using namespace erocket::frame_transforms;

/**
 * @brief Node that runs the controller for a 1-degree-of-freedom system
 */
class GenericControllerNode : public rclcpp::Node
{
public:
	explicit GenericControllerNode() : Node("generic_controller"),
    qos_profile_{rmw_qos_profile_sensor_data},
    qos_{rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, 5), qos_profile_)},

    vehicle_constants_{std::make_shared<VehicleConstants>(this)},
    state_aggregator_{std::make_unique<StateAggregator>(this, qos_)},
    setpoint_aggregator_{std::make_unique<SetpointAggregator>(this, qos_)},
    allocator_{std::make_unique<Allocator>(this, qos_, vehicle_constants_)},
    generic_controller_{this, qos_, state_aggregator_, setpoint_aggregator_, vehicle_constants_},

    flight_mode_{FlightMode::INIT},
    flight_mode_get_subscriber_{this->create_subscription<FlightMode>(
        FLIGHT_MODE_GET_TOPIC, qos_, 
        [this](const FlightMode::SharedPtr msg) {
            flight_mode_ = msg->flight_mode;
        }
    )}
    {
        double controller_freq = this->get_parameter(CONTROLLER_GENERIC_FREQUENCY_HERTZ_PARAM).as_double();
        double controller_period = 1.0 / controller_freq;

        // Controller timer at fixed frequency of controller
        controller_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(controller_period), 
            std::bind(&GenericControllerNode::controller_callback, this)
        );
	}

private:
    rmw_qos_profile_t qos_profile_;
    rclcpp::QoS qos_;

    std::shared_ptr<VehicleConstants> vehicle_constants_;
    std::shared_ptr<StateAggregator> state_aggregator_;
    std::shared_ptr<SetpointAggregator> setpoint_aggregator_;
    std::unique_ptr<Allocator> allocator_;
    GenericController generic_controller_;

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
void GenericControllerNode::controller_callback()
{
    if (flight_mode_ < FlightMode::ARM) {
        allocator_->compute_allocation_neutral();
        return;
    }

    else if (flight_mode_ == FlightMode::ARM) {
        static rclcpp::Time t0 = this->get_clock()->now();
        rclcpp::Time now = this->get_clock()->now();

        if (now - t0 < 3.0s) {
            auto max_servo_tilt_angle_radians = degrees_to_radians(vehicle_constants_->servo_max_tilt_angle_degrees_);
            auto inner_servo_tilt_angle_radians = sin(2.0 * M_PI * (now - t0).seconds()) * max_servo_tilt_angle_radians;
            auto outer_servo_tilt_angle_radians = sin(2.0 * M_PI * (now - t0).seconds() + M_PI_2) * max_servo_tilt_angle_radians;

            allocator_->indirect_actuation(
                inner_servo_tilt_angle_radians,
                outer_servo_tilt_angle_radians
            );
        }

        if (now - t0 > 4.0s) {
            // Allocate motor thrust based on the computed torque
            allocator_->indirect_actuation(
                0.0f,
                0.0f,
                0.0f,
                0.0f
            );
        }
        return;
    }

    // only run controller when in mission
    else if (flight_mode_ == FlightMode::TAKE_OFF || flight_mode_ == FlightMode::IN_MISSION || flight_mode_ == FlightMode::LANDING) {
        auto allocator_input = generic_controller_.compute();
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
	rclcpp::spin(std::make_shared<GenericControllerNode>());

	rclcpp::shutdown();
	return 0;
}
