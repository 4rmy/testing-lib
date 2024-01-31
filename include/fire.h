#include "liblvgl/lvgl.h"
#include <string>
#include <vector>
#include "api.h"

namespace fire {
    inline pros::Controller cont(pros::E_CONTROLLER_MASTER);
    inline int delay = 10;

    // lcd class to manage all methods related to the screen
    class lcd {
        // tell if the screen has been setup and ready to use
        inline static bool initialized = false;
        // screen to draw to
        inline static lv_obj_t *console = nullptr;
        // label for text
        inline static lv_obj_t *console_text = nullptr;

        inline static std::string display[14] = {"","","","","","","","","","","","","",""};

        public:
            // set up the screen to draw
            static void initialize();

            // allow printing to the screen
            static void print(std::string str);

            // clear the screen
            static void clear();
    };

    // drive class to manage all robot movement
    class drive {
        std::vector<pros::Motor> left_drive;
        std::vector<pros::Motor> right_drive;

        public:
            drive(
                std::vector<int> left_motor_ports,
                std::vector<int> right_motor_ports
            ) {
                for (int i = 0; i < left_motor_ports.size(); i++) {
                    left_drive.push_back(pros::Motor(i));
                }
                for (int i = 0; i < right_motor_ports.size(); i++) {
                    right_drive.push_back(pros::Motor(i));
                }
            };

            void tank_control();
            void split_arcade();
            void split_arcade_flipped();
    };
}