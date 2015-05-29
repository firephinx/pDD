/**
* This file is part of pdd.
*
* This is the main file that will begin the program.
*
* Written by Kevin Zhang and with help from rock and LSD-SLAM
*
*/

#include "pdd.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFunctions.h"
#include "pddSystem.h"

#include "IOWrapper/ROS/ROSImageStreamThread.h"
#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
#include "IOWrapper/ROS/rosReconfigure.h"

#include <X11/Xlib.h>

#include <fstream>
#include <vector>
#include <numeric>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/program_options.hpp>

#include "units.h"
#include "utils.h"
#include "mesh.h"
#include "model.h"

using namespace pDD;

int main(int argc, char** argv){

    XInitThreads(); 

    ros::init(argc, argv, "pdd");

    dynamic_reconfigure::Server<pdd::LSDParamsConfig> srv(ros::NodeHandle("~"));
    srv.setCallback(dynConfCb);

    dynamic_reconfigure::Server<pdd::LSDDebugParamsConfig> srvDebug(ros::NodeHandle("~Debug"));
    srvDebug.setCallback(dynConfCbDebug);

    packagePath = ros::package::getPath("pdd")+"/";

    InputImageStream* inputStream = new ROSImageStreamThread();

    std::string calibFile;
    if(ros::param::get("~calib", calibFile))
    {
        ros::param::del("~calib");
        inputStream->setCalibration(calibFile);
    }
    else
        inputStream->setCalibration("");
    inputStream->run();

    Output3DWrapper* outputWrapper = new ROSOutput3DWrapper(inputStream->width(), inputStream->height());
    pdd pddNode(inputStream, outputWrapper);
    slamNode.Loop();



    if (inputStream != nullptr)
        delete inputStream;
    if (outputWrapper != nullptr)
        delete outputWrapper;

    /*
	// Options Parser
    bpo::options_description desc("Modeller Options");
    desc.add_options()
        ("help,h", "Print this help message")
        ("mesh,m", po::value< string >()->required(), "Mesh .obj file")
        ("textures,t", po::value< vector<string> >()->multitoken()->required(),
                "Texture image files")
        ("outputpath,o", po::value< string >(),
                "The output folder for the new model and debug information")
        ("name,n", po::value< string >()->required(),
                "The name of the new model")
        ("billboardsizes,s", po::value< vector< double > >()->multitoken(),
                "Size of each binary patch per layer")
        //("billboards,b", po::value< vector< int > >()->multitoken(),
        //        "Number of patches per layer")
        ("billboard-density,e", po::value<float>()->default_value(4.0),
                "The density of patches on the object surface.")
        ("orb-prior,p", "Builds an ORB prior in addition to a ROCK model.")
        ("num-features,f", po::value<int>()->default_value(10),
                "The number of features to find for the ORB prior.")
        ("scale-factor,c", po::value<float>()->default_value(1.2),
                "The scale factor of the pyramid used for ORB detection.")
        ("num-levels,l", po::value<int>()->default_value(12),
                "The number of pyramid levels used for ORB detection.")
        ("debug,d", "Creates extra debug images")
    ;
    
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    // if the help flag was specified, print the argument options and return.
    if(vm.count("help")){
        cout << desc << endl;
        return 1;
    }
    
    string output_path = "./";
    if(vm.count("outputpath")){
        output_path = vm["outputpath"].as<string>();
    }
    
    string name = vm["name"].as<string>();
    
    bool debug = vm.count("debug");
    
    vector<cm> billboard_sizes;
    if(vm.count("billboardsizes")){
        billboard_sizes = vm["billboardsizes"].as< vector<double> >();
    }
    else{
        // default billboard sizes
        //billboard_sizes.push_back(0.01);
        billboard_sizes.push_back(0.00707);
        billboard_sizes.push_back(0.005);
        billboard_sizes.push_back(0.00354);
        billboard_sizes.push_back(0.0025);
        //billboard_sizes.push_back(0.00177);
        //billboard_sizes.push_back(0.00125);
    } */
    
    /* was commented before
    vector<int> billboards;
    if(vm.count("billboards")){
        billboards = vm["billboards"].as< vector<int> >();
    }
    else{
        // default layer patches
        //billboards.push_back(1024);
        billboards.push_back(2048);
        billboards.push_back(4096);
        billboards.push_back(8192);
        billboards.push_back(16384);
        //billboards.push_back(32768);
        //billboards.push_back(65536);
    }
    */
    
    /* was commented before
    if(billboard_sizes.size() != billboards.size()){
        cout << "The number of entries in the 'billboards' argument "
             << "must be the same as the number of entries in the "
             << "'billboardsizes' argument.  "
             << billboards.size() << "/" << billboard_sizes.size() << endl;
        return -1;
    }
    */
    
    /*
    string mesh_path = vm["mesh"].as<string>();
    vector<string> texture_names;
    texture_names = vm["textures"].as<vector<string> >();
    float billboard_density = vm["billboard-density"].as<float>();
    //string texture_name = vm["textures"].as<string>();
    //texture_names.push_back(texture_name);
    
    Mesh mesh;
    mesh.ReadObj(mesh_path);
    */
    
    /* was commented before
    vector<cm> areas;
    mesh.TriangleAreas(areas);
    float total_area = std::accumulate(areas.begin(), areas.end(), 0.);
    cout << total_area << endl;
    cout << areas[0] << " " << areas[1] << " " << areas[2] << endl;
    return 0;
    */

    /*
    vector<cv::Mat> textures;
    for(size_t i = 0; i < texture_names.size(); ++i){
        cv::Mat texture;
        
        // This is confusing
        //texture = cv::imread(vm["textures"].as<string>(),
        //            CV_LOAD_IMAGE_UNCHANGED);
        
        texture = cv::imread(texture_names[i], CV_LOAD_IMAGE_UNCHANGED);
        textures.push_back(texture);
        
        mesh.SetUvNormalization(texture.rows, texture.cols, i);
    }
    
    mesh.textures_ = textures;
    
    float superpixel_compactness = 20.;
    Model model(mesh,
                textures,
                //billboards,
                superpixel_compactness,
                billboard_sizes,
                billboard_density,
                5.,
                debug);
    
    stringstream model_path;
    model_path << output_path;
    //if(output_path[output_path.length()-1] != "/"){
    //    model_path << "/";
    //}
    model_path << name << ".yml";
    model.Write(model_path.str(), debug);
    
    if(debug){
        model.GenerateDebugInfo(output_path);
    } */
    
    /* was commented before
    if(vm.count("orb-prior")){
        PointFeaturePrior prior(mesh,
                                textures,
                                vm["num-features"].as<int>(),
                                vm["scale-factor"].as<float>(),
                                vm["nm-levels"].as<int>());
        
        prior.Write(prior_path.str());
    }
    */
    return 0;
}
