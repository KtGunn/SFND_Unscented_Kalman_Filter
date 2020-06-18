/* \author Aaron Brown */
// Handle logic for creating traffic on highway and animating it

#include "render/render.h"
#include "sensors/lidar.h"
#include "tools.h"

#include<pcl/visualization/pcl_plotter.h>
#include<random>


// Structure to hold error data
// We'll record and plot the error of speed (v) and
// yaw rate. These are assumed constant and the filter
// has to track their changes based on a noise model
// It'll be interesting to see how weel that works
// 
struct sError {
    std::vector<double> eV; // Speed error
    std::vector<double> eR; // Yaw rate error
};

class Highway
{
 public:
    
    std::vector<sError> ukfError;

    std::vector<Car> traffic;
    Car egoCar;
    Tools tools;
    bool pass = true;
    std::vector<double> rmseThreshold = {0.30,0.16,0.95,0.70};
    std::vector<double> rmseFailLog = {0.0,0.0,0.0,0.0};

    Lidar* lidar;
    
    // Plotters for NIS
    std::vector<pcl::visualization::PCLPlotter*> vPlts = {nullptr,nullptr, nullptr};

    // Parameters 
    // --------------------------------
    // Set which cars to track with UKF
    std::vector<bool> trackCars = {true,true,true};

    // Visualize sensor measurements
    bool visualize_lidar = true;
    bool visualize_radar = true;
    bool visualize_pcd = false;

    // Predict path in the future using UKF
    double projectedTime = 0;
    int projectedSteps = 0;
    // --------------------------------
    
    //////////////////////////////////////////////////////////////
    /// Select the active car
    //
    void SetCar (const int id) {
	for (short n=0; n<trackCars.size(); n++)
	    trackCars[n] = false;
	trackCars[id] = true;
	return;
    }

    //////////////////////////////////////////////////////////////
    /// Process noise variance
    //
    void Variance (const std::string& type, const double value ) {
	if (value < 0)
	    return;
	for (short n=0; n<traffic.size(); n++)
	    if (type.compare("rate") == 0)
		traffic[n].ukf.RateVariance (value);
	    else if (type.compare("acc") == 0)
		traffic[n].ukf.AccVariance (value);
	    else
		throw (101);

	return;
    }

    //////////////////////////////////////////////////////////////
    /// Process noise variance
    //
    void Update (const bool value) {
	for (short n=0; n<traffic.size(); n++)
	    traffic[n].ukf.Update (value);
	return;
    }

    //////////////////////////////////////////////////////////////
    /// (de-)Activate Sensor
    //
    void SetSensor (const std::string sensor) {

	bool lidarOn = false;
	bool radarOn = false;

	if ( sensor.compare ("lidar") == 0 ) {
	    lidarOn = true;
	} else {
	    radarOn = true;
	}

	//  Note that all cars and ukf's are defined
	traffic[0].ukf.LidarState(lidarOn);
	traffic[0].ukf.RadarState(radarOn);
	
	traffic[1].ukf.LidarState(lidarOn);
	traffic[1].ukf.RadarState(radarOn);
	
	traffic[2].ukf.LidarState(lidarOn);
	traffic[2].ukf.RadarState(radarOn);

	return;
    }
    

    
    Highway(pcl::visualization::PCLVisualizer::Ptr& viewer) {
	
	tools = Tools();
	
	egoCar = Car(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), 0, 0, 2, "egoCar");
	
	Car car1(Vect3(-10, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), 5, 0, 2, "car1");
	
	std::vector<accuation> car1_instructions;
	accuation a = accuation(0.5*1e6, 0.5, 0.0);
	car1_instructions.push_back(a);
	a = accuation(2.2*1e6, 0.0, -0.2);
	car1_instructions.push_back(a);
	a = accuation(3.3*1e6, 0.0, 0.2);
	car1_instructions.push_back(a);
	a = accuation(4.4*1e6, -2.0, 0.0);
	car1_instructions.push_back(a);
	
	car1.setInstructions(car1_instructions);
	if( trackCars[0] )
	    {
		UKF ukf1;
		car1.setUKF(ukf1);
	    }
	traffic.push_back(car1);
	ukfError.push_back(sError());
	
	Car car2(Vect3(25, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), -6, 0, 2, "car2");
	std::vector<accuation> car2_instructions;
	a = accuation(4.0*1e6, 3.0, 0.0);
	car2_instructions.push_back(a);
	a = accuation(8.0*1e6, 0.0, 0.0);
	car2_instructions.push_back(a);
	car2.setInstructions(car2_instructions);
	if( trackCars[1] )
	    {
		UKF ukf2;
		car2.setUKF(ukf2);
	    }
	traffic.push_back(car2);
	ukfError.push_back(sError());
	
	Car car3(Vect3(-12, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), 1, 0, 2, "car3");
	std::vector<accuation> car3_instructions;
	a = accuation(0.5*1e6, 2.0, 1.0);
	car3_instructions.push_back(a);
	a = accuation(1.0*1e6, 2.5, 0.0);
	car3_instructions.push_back(a);
	a = accuation(3.2*1e6, 0.0, -1.0);
	car3_instructions.push_back(a);
	a = accuation(3.3*1e6, 2.0, 0.0);
	car3_instructions.push_back(a);
	a = accuation(4.5*1e6, 0.0, 0.0);
	car3_instructions.push_back(a);
	a = accuation(5.5*1e6, -2.0, 0.0);
	car3_instructions.push_back(a);
	a = accuation(7.5*1e6, 0.0, 0.0);
	car3_instructions.push_back(a);
	car3.setInstructions(car3_instructions);
	if( trackCars[2] )
	    {
		UKF ukf3;
		car3.setUKF(ukf3);
	    }
	traffic.push_back(car3);
	ukfError.push_back(sError());
	
	lidar = new Lidar(traffic,0);
	
	// render environment
	renderHighway(0,viewer);
	egoCar.render(viewer);
	car1.render(viewer);
	car2.render(viewer);
	car3.render(viewer);
    }
    
    // Returns 'true' if Test interval is concluded, else 'false'
    
    const bool stepHighway(double egoVelocity, long long timestamp, int frame_per_sec,
		     pcl::visualization::PCLVisualizer::Ptr& viewer,const bool doNis)
    {
	// We set the flag 'ture' when test period is over
	bool returnFlag = false;
	
	if (visualize_pcd) {
	    pcl::PointCloud<pcl::PointXYZ>::Ptr trafficCloud =
		tools.loadPcd("../src/sensors/data/pcd/highway_"+std::to_string(timestamp)+".pcd");
	    renderPointCloud(viewer, trafficCloud, "trafficCloud", Color((float)184/256,(float)223/256,(float)252/256));
	}

	// render highway environment with poles
	renderHighway(egoVelocity*timestamp/1e6, viewer);
	egoCar.render(viewer);
	
	for (int i = 0; i < traffic.size(); i++) {
	    traffic[i].move((double)1/frame_per_sec, timestamp);
	    if (!visualize_pcd)
		traffic[i].render(viewer);

	    // Sense surrounding cars with lidar and radar
	    if (trackCars[i]) {

		// Ground truth
		VectorXd gt(4);
		gt << traffic[i].position.x, traffic[i].position.y, traffic[i].velocity*cos(traffic[i].angle),
		    traffic[i].velocity*sin(traffic[i].angle);

		tools.ground_truth.push_back(gt);
		tools.lidarSense(traffic[i], viewer, timestamp, visualize_lidar);
		tools.radarSense(traffic[i], egoCar, viewer, timestamp, visualize_radar);
		tools.ukfResults(traffic[i],viewer, projectedTime, projectedSteps);

		VectorXd estimate(4);
		double v  = traffic[i].ukf.x_(2);
		double yaw = traffic[i].ukf.x_(3);
		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;
		estimate << traffic[i].ukf.x_[0], traffic[i].ukf.x_[1], v1, v2;
		tools.estimations.push_back(estimate);
		
		// Accumlate the errors
		double foo = traffic[i].velocity - traffic[i].ukf.x_(2);
		ukfError[i].eV.push_back (traffic[i].velocity - traffic[i].ukf.x_(2));
		ukfError[i].eR.push_back (traffic[i].yawRate - traffic[i].ukf.x_(4));

		if (doNis) {
		    NIS (i);
		}
	    }
	}

	viewer->addText("Accuracy - RMSE:", 30, 300, 20, 1, 1, 1, "rmse");
	VectorXd rmse = tools.CalculateRMSE(tools.estimations, tools.ground_truth);
	viewer->addText(" X: "+std::to_string(rmse[0]), 30, 275, 20, 1, 1, 1, "rmse_x");
	viewer->addText(" Y: "+std::to_string(rmse[1]), 30, 250, 20, 1, 1, 1, "rmse_y");
	viewer->addText("Vx: "	+std::to_string(rmse[2]), 30, 225, 20, 1, 1, 1, "rmse_vx");
	viewer->addText("Vy: "	+std::to_string(rmse[3]), 30, 200, 20, 1, 1, 1, "rmse_vy");
	
	if (timestamp > 1.0e6) {
	    // Test time is up!
	    returnFlag = true;

	    if(rmse[0] > rmseThreshold[0])
		{
		    rmseFailLog[0] = rmse[0];
		    pass = false;
		}
	    if(rmse[1] > rmseThreshold[1])
		{
		    rmseFailLog[1] = rmse[1];
		    pass = false;
		}
	    if(rmse[2] > rmseThreshold[2])
		{
		    rmseFailLog[2] = rmse[2];
		    pass = false;
		}
	    if(rmse[3] > rmseThreshold[3])
		{
		    rmseFailLog[3] = rmse[3];
		    pass = false;
		}
	}
	if(!pass) {
	    viewer->addText("RMSE Failed Threshold", 30, 150, 20, 1, 0, 0, "rmse_fail");
	    if(rmseFailLog[0] > 0)
		viewer->addText(" X: "+std::to_string(rmseFailLog[0]), 30, 125, 20, 1, 0, 0, "rmse_fail_x");
	    if(rmseFailLog[1] > 0)
		viewer->addText(" Y: "+std::to_string(rmseFailLog[1]), 30, 100, 20, 1, 0, 0, "rmse_fail_y");
	    if(rmseFailLog[2] > 0)
		viewer->addText("Vx: "+std::to_string(rmseFailLog[2]), 30, 75, 20, 1, 0, 0, "rmse_fail_vx");
	    if(rmseFailLog[3] > 0)
		viewer->addText("Vy: "+std::to_string(rmseFailLog[3]), 30, 50, 20, 1, 0, 0, "rmse_fail_vy");
	}

	return (returnFlag);
    }
    
    void KeepAlive () {

	// Plot NIS for active cars
	for (int i = 0; i < traffic.size(); i++) {
	    if (trackCars[i]) {
		NIS (i);
		Errors (i);
	    }
	}
	return;
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// PLOT estimation errors
    //
    void Errors (const int carID) {
	
	std::string title = "Estimation Error Car #" + std::to_string (1+carID);
	pcl::visualization::PCLPlotter* ePlots = new pcl::visualization::PCLPlotter(title.c_str());
	
	ePlots->setBackgroundColor (62.0/255.0, 164.0/255.0, 216.0/255.0); // blue
	ePlots->setXRange(0.0, 300.0);
	ePlots->setYRange(-2.0, 2.0);
	ePlots->setWindowSize (500,275);

	ePlots->setXTitle("Update count");
	ePlots->setYTitle("Error = Ground truth - Estimate");
	ePlots->setShowLegend(true);

	// this does not work!
	ePlots->setWindowPosition(800+100*carID, 300*(carID-1));
	
	// Speed error
	std::vector<double> vIndices;
	for (short n=0; n<ukfError[carID].eV.size(); n++) {
	    vIndices.push_back ((double)n);
	}
	ePlots->addPlotData (vIndices, ukfError[carID].eV,"Speed [m/s]");
	
	// Turn rate error
	vIndices.clear();
	for (short n=0; n<ukfError[carID].eR.size(); n++) {
	    vIndices.push_back ((double)n);
	}
	ePlots->addPlotData (vIndices, ukfError[carID].eR,"Turn rate [rad/s]");

	ePlots->spinOnce(1); // Bring up the plot
	
	return;
    }

    void NIS (const int carID) {
	
	/****** SANITY check ******/
	if ( !trackCars[carID] ) {
	    std::cerr << "NIS called for cart that is not active!\n";
	    return;
	}
	
	//////////////////////////////////////////////////////////////////
	/// Create plotter if first time called
	//
	if ( vPlts[carID] == nullptr ) {
	    std::string title = "NIS Car #" + std::to_string (1+carID);
	    vPlts[carID] = new pcl::visualization::PCLPlotter(title.c_str());

	    // Very cool bg colour
	    vPlts[carID]->setBackgroundColor (0.5, 0.23, 0.1);    
	    vPlts[carID]->setColorScheme (11);

	    vPlts[carID]->setWindowSize(500,275);
	    vPlts[carID]->setXRange(0.0, 600.0);
	    vPlts[carID]->setYRange(0.0, 20.0);

	    vPlts[carID]->setShowLegend(true);
	    vPlts[carID]->setXTitle("Update count (doubled)");
	    vPlts[carID]->setYTitle("NIS");

	    // this does not work!
	    vPlts[carID]->setWindowPosition(400+100*carID, 300*(carID-1));
	}

	vPlts[carID]->clearPlots ();

	if (traffic[carID].ukf.use_laser_) {
	    std::vector<double> vIndices;
	    for (short n=0; n<traffic[carID].ukf.vNIS_lidar_.size(); n++) {
		vIndices.push_back( (double)(2*n));
	    }
	    std::vector<char> vColor{(char)0, (char)200, (char)200, (char)255};
	    vPlts[carID]->addPlotData (vIndices, traffic[carID].ukf.vNIS_lidar_, "Lidar",0, vColor);

	    double aX[2] = {0.0, 600.0};    // Add the ki-squared limit
	    double aY[2] = {6.0, 6.0};      // 2-df approx 6.0 @ 95%
	    
	    vPlts[carID]->addPlotData (aX, aY, 2,"Lidar 95%");
	    
	}

	if (traffic[carID].ukf.use_radar_) {
	    std::vector<double> vIndices;
	    for (short n=0; n<traffic[carID].ukf.vNIS_radar_.size(); n++) {
		vIndices.push_back( (double)(2*n+1));
	    }
	    std::vector<char> vColor{(char)255, (char)255, (char)0, (char)255};
	    vPlts[carID]->addPlotData (vIndices, traffic[carID].ukf.vNIS_radar_,"Radar", 0, vColor);

	    double aX[2] = {0.0, 600.0};    // Add the ki-squared limit
	    double aY[2] = {7.8, 7.8};      // 3-df approx 7.8 @ 95%
	    
	    vPlts[carID]->addPlotData (aX, aY, 2, "Radar 95%");

	}
	
	vPlts[carID]->spinOnce(1);

	return;
    }
};
