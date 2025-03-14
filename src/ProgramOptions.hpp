#ifndef PROGRAM_OPTIONS_HPP
#define PROGRAM_OPTIONS_HPP

#include <string>
#include <list>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <getopt.h>
#include <yaml-cpp/yaml.h>

struct ProgramOptions
{
    std::string input_file;
    std::string output_file;

    double radius;

    int max_orphan_per_voxel;
    int reading_per_batch;
    
    bool policy_main_mesh;
    int policy_main_mesh_activation_batch_number;
};

ProgramOptions parseConfigFile(const std::string &config_file)
{
    ProgramOptions options;
    
    // Parse YAML config file
    std::ifstream fin(config_file);
    if (!fin) {
        std::cerr << "Failed to open config file: " << config_file << std::endl;
        exit(EXIT_FAILURE);
    }

    YAML::Node config = YAML::LoadFile(config_file);

    // Helper function to check and report missing options
    auto checkRequired = [](const YAML::Node& node, const std::string& name) {
        if (!node.IsDefined()) {
            throw std::runtime_error("Required configuration option '" + name + "' is missing");
        }
    };

    // Read radius value
    checkRequired(config["radius"], "radius");
    options.radius = config["radius"].as<double>();

    checkRequired(config["input_file"], "input_file");
    options.input_file = config["input_file"].as<std::string>();

    checkRequired(config["output_file"], "output_file");
    options.output_file = config["output_file"].as<std::string>();

    checkRequired(config["max_orphan_per_voxel"], "max_orphan_per_voxel");
    options.max_orphan_per_voxel = config["max_orphan_per_voxel"].as<int>();

    checkRequired(config["reading_per_batch"], "reading_per_batch");
    options.reading_per_batch = config["reading_per_batch"].as<int>();

    // For nested options
    checkRequired(config["one_mesh_policy"], "one_mesh_policy");
    checkRequired(config["one_mesh_policy"]["enabled"], "one_mesh_policy.enabled");
    options.policy_main_mesh = config["one_mesh_policy"]["enabled"].as<bool>();

    checkRequired(config["one_mesh_policy"]["activation_batch_number"], "one_mesh_policy.activation_batch_number");
    options.policy_main_mesh_activation_batch_number = config["one_mesh_policy"]["activation_batch_number"].as<int>();
    
    return options;
}

ProgramOptions parseCommandLine(int argc, char **argv)
{
    ProgramOptions options;
    int c;

    bool config_file_provided = false;
    
    // Check if -c option exists
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-c") == 0) {
            config_file_provided = true;
            break;
        }
    }

    if (config_file_provided) {
        // Check for config file first
        while ((c = getopt(argc, argv, "c:i:o:r:p")) != -1) {
            if (c == 'c') {
                // If config file specified, parse it and return those options
                return parseConfigFile(optarg);
            }
        }
    }
    else 
    {
        // Reset optind to parse arguments again
        optind = 1;
        
        // Parse command line arguments
        while ((c = getopt(argc, argv, "c:i:o:r")) != -1)
        {
            switch (c)
            {
            case 'i':
                options.input_file = optarg;
                break;
            case 'o':
                options.output_file = optarg;
                break;
            case 'r':
            {
                std::istringstream iss(optarg);
                double radius;
                while (iss >> radius)
                {
                    options.radius = radius; // Take the first radius
                    break;
                }
                break;
            }
            case 'c':
                break; // Already handled
            default:
                std::cerr << "Unknown option: " << static_cast<char>(c) << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    }
    return options;
}

#endif // PROGRAM_OPTIONS_HPP