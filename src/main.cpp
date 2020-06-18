/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors


#include "highway.h"

int main(int argc, char** argv)
{
    
    ////////////////////////////////////////////////////////////
    /// USAGE
    //
    std::cerr << "**********************************************************************\n";
    std::cerr << "usage: ./ukf_highway [car=<1,2,3>] [sensor=<lidar,radar>] [varAcc=<double>] [varRate=<double>] [std_update]\n";
    std::cerr << "arguments are optional.\n";
    std::cerr << " To specify all cars, skip 'car=' on the command line.\n";
    std::cerr << " Or select one car, e.g. 'car=1', for car 1.\n";
    std::cerr << " To specify both sensors, skip 'sensor=' on the command line.\n";
    std::cerr << " Or select one sensor, e.g. 'sensor=lidar'.\n";
    std::cerr << " Process noise variance can optionally be specified also,\n";
    std::cerr << " 'varAcc' for acceleration and 'varRate' for turn rate.\n";
    std::cerr << " 'varAcc' for acceleration and 'varRate' for turn rate.\n";
    std::cerr << " 'std_update' uses standard Kalman update in place of sigma estimation\n";
    

    ////////////////////////////////////////////////////////////
    /// COMMAND line arguments processing
    //
    std::string aOptions[5] = {"car=", "sensor=", "varAcc=", "varRate=", "std_update"};
    
    // defaults
    std::string activeSensors = "both";
    bool std_kalman_update = false;
    double rate_variance = -1;
    double acc_variance = -1;
    int activeCars = 0;

    if (argc > 1)
	std::cerr << "\n\n Processing user input\n:";
    else 
	std::cerr << "\n\n";
    
    for (short n=1; n<argc; n++) {
	std::string option (argv[n]);
	std::cout << " arg n=" << option << std::endl;

	short pos = 0;
	if ((pos=option.find (aOptions[0])) == 0) {
	    // car
	    std::string cars=option.substr(pos+aOptions[0].length());
	    int carID = std::stoi(cars);
	    if (carID <1 || carID >3) {
		std::cerr << "wrong input: '" << option << "' is wrong. Bye...\n";
		return (-1);
	    } else {
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++
		// Signal that user has selected a car
		activeCars = carID;
	    }
	    continue;
	}
	if ((pos=option.find (aOptions[1])) == 0) {
	    // sensor
	    std::string sensor=option.substr(pos+aOptions[1].length());
	    std::cout << " sensor=" << sensor << " pos=" << pos << " l=" << option.length() << std::endl;
	    if (sensor.compare("lidar") == 0 || sensor.compare("radar") == 0 ) {
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++
		// Signal that user has selected a sensor type
		activeSensors = sensor;

	    } else {
		std::cerr << "wrong input for 'sensor='. Bye...\n";
		std::cerr << "wrong input: '" << option << "' is wrong. Bye...\n";
		return (-1);
	    }
	    continue;
	}
	if ((pos=option.find (aOptions[2])) == 0) {
	    // acceleration process noise
	    std::string varAcc=option.substr(pos+aOptions[2].length());
	    acc_variance = std::stod(varAcc);
	    std::cout << " Accerleration variance = " << acc_variance << std::endl;
	    continue;
	}
	if ((pos=option.find (aOptions[3])) == 0) {
	    // yaw rate process noise
	    std::string varRate=option.substr(pos+aOptions[3].length());
	    rate_variance = std::stod(varRate);
	    std::cout << " Rate variance = " << rate_variance  << std::endl;
	    continue;
	}
	if ((pos=option.find (aOptions[4])) == 0) {
	    // standard Kalman update
	    std::cout << " Standard Kalman update to be used\n";
	    std_kalman_update = true;
	    continue;
	}
	
	std::cout << std::endl << "`" << option << "` is not recognized. Sorry...\n\n";
	return 0;
    }


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    float x_pos = 0;
    viewer->setCameraPosition ( x_pos-26, 0, 15.0, x_pos+25, 0, 0, 0, 0, 1);
    
    Highway highway(viewer);

    
    ////////////////////////////////////////////////////////////////////
    /// HANDLE user selections if any
    //
    if ( activeSensors.compare ("both") != 0 ) {
	highway.SetSensor (activeSensors);
    }
    if ( activeCars != 0 ) {
	// cars=1,2,3 but index into 'traffic' is 0,1,2
	highway.SetCar (activeCars-1);
    }
    
    highway.Variance ("acc", acc_variance ); // Value will be checked
    highway.Variance ("rate", rate_variance );
    highway.Update ( std_kalman_update );

    
    int frame_per_sec = 30;
    int sec_interval = 10;
    int frame_count = 0;
    int time_us = 0;
    
    double egoVelocity = 25;
    
    int downCounter = 10;
    bool done = false;
    int counter = 0;

    while (frame_count < (frame_per_sec*sec_interval) && !done) {
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
	
	/// Visualize the Normalized Innovation Squared index
	//  but only periodically
	bool doNis = false;
	if ( !downCounter-- ) {
	    doNis = true;
	    downCounter = 10;
	}

	done = highway.stepHighway(egoVelocity,time_us, frame_per_sec, viewer, doNis);
	viewer->spinOnce(1000/frame_per_sec);
	frame_count++;
	time_us = 1000000*frame_count/frame_per_sec;
	
	done = false;
	counter++;
    }

    std::cout << " Counter = " << counter << std::endl;
    highway.KeepAlive();

    char c = getchar();
    std::cout << "Hit return to exit\n";

    return (0);
}
