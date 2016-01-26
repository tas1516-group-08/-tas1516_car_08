// Contribution of Hans Boehme

#include "wii_lib.h"

wii_lib::wii_lib()
{
    /*Initilaization of publishers, subscribers and messages*/
    wii_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);

    wii_communication_pub = nh_.advertise<std_msgs::Int16MultiArray>("wii_communication",1);

    wii_sub_ = nh_.subscribe<wiimote::State>("wiimote/state",100,&wii_lib::wiiStateCallback,this);

    msg_Initialization(wii_state_);

    controlMode.data = 0;
    emergencyBrake.data = 1;
    b_pressed = false;
}

void wii_lib::wiiStateCallback(const wiimote::State::ConstPtr& wiiState)
{
    /*check if B button is pressed*/
    b_pressed = false;
    if(wiiState.get()->buttons[WII_BUTTON_B]==1)
    {
        b_pressed = true;             /*Generating Trajectory*/  
    }

    /*check if C button is pressed*/
    if(wiiState.get()->nunchuk_buttons[WII_BUTTON_NUNCHUK_C]==1)
    {
        controlMode.data = 1; 			/*setting controlMode flag to 1 --> Automatic Control!*/

        /*check if Z button is pressed*/
        if(wiiState.get()->nunchuk_buttons[WII_BUTTON_NUNCHUK_Z]==1)
        {
            emergencyBrake.data = 1; 		/*setting emergencyBrake flag to 1*/
        }
        else
        {
            emergencyBrake.data = 0; 		/*setting emergencyBrake flag to 0*/
        }
    }

    /*check if A button is pressed*/
    else if (wiiState.get()->buttons[4]==1)
    {
        controlMode.data = 2; 			/*setting controlMode flag to 2 --> Parking Control!*/
        
        /*check if Z button is pressed*/
        if(wiiState.get()->nunchuk_buttons[WII_BUTTON_NUNCHUK_Z]==1)
        {
            emergencyBrake.data = 1; 		/*setting emergencyBrake flag to 1*/
        }
        else
        {
            emergencyBrake.data = 0; 		/*setting emergencyBrake flag to 0*/
        }
    }
    
    /*If no button is pressed:*/
    else
    {
        controlMode.data = 0;			/*setting controlMode flag to 0 --> Manual Control!*/

        /*check if Z button is pressed*/
        if(wiiState.get()->nunchuk_buttons[WII_BUTTON_NUNCHUK_Z]==1)
        {
            emergencyBrake.data = 1; 		/*setting emergencyBrake flag to 1*/

            servo.x = 1500;
            servo.y = 1500;
        }
        else
        {
            emergencyBrake.data = 0; 		/*setting emergencyBrake flag to 0*/

            if(wiiState.get()->nunchuk_joystick_zeroed[1]>=0)
            {
                SCALE_FAKTOR_THROTTLE = 100; 	/*scale factor for driving forward*/
            }
            else
            {
                SCALE_FAKTOR_THROTTLE = 300; 	/*scale factor for driving reverse*/
            }

            /* mapping analog nunchuk state to servo command*/
            servo.x = 1500 + SCALE_FAKTOR_THROTTLE*wiiState.get()->nunchuk_joystick_zeroed[1];
            servo.y = 1500 + SCALE_FAKTOR_STEERING*wiiState.get()->nunchuk_joystick_zeroed[0];
        }

        wii_servo_pub_.publish(servo); 		/*publish servo messages to arduino*/
    }

    wii_state_.data[0] = controlMode.data;
    wii_state_.data[1] = emergencyBrake.data;
}

void wii_lib::msg_Initialization(std_msgs::Int16MultiArray &msg)
{
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 2;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "flag";
    msg.data.clear();
    msg.data.resize(2);
}

