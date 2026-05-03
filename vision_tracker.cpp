#include "vision_tracker.hpp"
#include "pros/distance.hpp"
#include "pros/vision.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

pros::MotorGroup leftMotors5({-19, -18}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors5({9, 10}, pros::MotorGearset::blue);

pros::AIVision aivision(15);
pros::Distance trackerDistance(11);

pros::AIVision::Color yellow = {
    .id = 2,
    .red = 255,
    .green = 215,
    .blue = 60,
    .hue_range = 10,
    .saturation_range = 0.5
};
pros::AIVision::Color blue = {
    .id = 1, 
    .red = 32, 
    .green = 142, 
    .blue = 194, 
    .hue_range = 20, 
    .saturation_range = 0.5
};




struct PID {
    double kp, ki, kd;
    double prev_error = 0;
    double integral = 0;
    static constexpr double I_CLAMP = 200.0;

    double update(double error) {
        integral += error;
        integral = std::clamp(integral, -I_CLAMP, I_CLAMP);
        double derivative = error - prev_error;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }

    void reset() {
        prev_error = 0;
        integral = 0;
    }
};


static PID turn_pid = { .kp = 0.2, .ki = 0.0, .kd = 5 };
static PID fwd_pid  = { .kp = 0.7, .ki = 0.0, .kd = 7 };

static constexpr int X_CENTER = 150;   
static constexpr int DEADBAND = 18;

static constexpr double MAX_TURN = 80.0;
static constexpr double MAX_FWD  = 100.0;

static constexpr double LEFT_SCALE  = 1.0; 
static constexpr double RIGHT_SCALE = 1.0;

static constexpr double TARGET_HEIGHT = 130;

static void drive(double left, double right) {
    leftMotors5.move((int32_t)(left * LEFT_SCALE));
    rightMotors5.move((int32_t)(right * RIGHT_SCALE));
}

bool tracker_enabled = true;
static pros::Task* tracker_task = nullptr;

static void tracker_fn(void*) {

    aivision.enable_detection_types(pros::AivisionModeType::colors);
    aivision.set_color(blue);
    //aivision.set_tag_family(pros::v5::AivisionTagFamily::tag_16H5, true);

    while (true) {

        if (!tracker_enabled) {
            drive(0, 0);
            turn_pid.reset();
            fwd_pid.reset();
            pros::delay(20);
            continue;
        }

        auto objects = aivision.get_all_objects();
        bool found = false;

        for (auto& obj : objects) {

            if (!pros::AIVision::is_type(obj, pros::AivisionDetectType::color))
                continue;

            found = true;

            double raw_x = obj.object.element.xoffset+ (obj.object.element.width/2);
            double xoffset = raw_x - X_CENTER;

            double turn_pwr = 0;

            if (std::abs(xoffset) > DEADBAND) {
                turn_pwr = 1.1*turn_pid.update(xoffset);
                turn_pwr = std::clamp(turn_pwr, -MAX_TURN, MAX_TURN);
            } else {
                turn_pid.reset();
            }

            double height = obj.object.element.height;
            double error = TARGET_HEIGHT - height;

            double fwd_pwr = fwd_pid.update(error);
            fwd_pwr = std::clamp(fwd_pwr, 0.0, MAX_FWD);

            if (trackerDistance.get_distance() < 200) {
                fwd_pwr = 0;
            }

            
            double left  = fwd_pwr + turn_pwr;
            double right = fwd_pwr - turn_pwr;

            
            double max_pwr = std::max(std::abs(left), std::abs(right));
            if (max_pwr > 127.0) {
                left  = left  / max_pwr * 127.0;
                right = right / max_pwr * 127.0;
            }

            drive(left, right);
            break;
        }

       static int lost_frames = 0;

if (!found) {
    lost_frames++;

    if (lost_frames > 5) { 
        drive(0, 0);
        turn_pid.reset();
        fwd_pid.reset();
    }
} else {
    lost_frames = 0;
}

        pros::delay(20);
    }
}

void vision_tracker_init() {
    if (tracker_task == nullptr) {
        tracker_task = new pros::Task(tracker_fn, nullptr, "vision_tracker");
    }
}

void vision_tracker_enable(bool enable) {
    tracker_enabled = enable;
}