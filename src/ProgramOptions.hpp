#ifndef PROGRAM_OPTIONS_HPP
#define PROGRAM_OPTIONS_HPP

#include <string>
#include <list>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <getopt.h>

struct ProgramOptions {
    std::string input_infiles;
    std::string outfile;
    std::list<double> radii;
    bool parallel_flag = false;
};

ProgramOptions parseCommandLine(int argc, char** argv) {
    ProgramOptions options;
    int c;

    while ((c = getopt(argc, argv, "i:o:r:p")) != -1) {
        switch (c) {
            case 'i':
                options.input_infiles = optarg;
                break;
            case 'o':
                options.outfile = optarg;
                break;
            case 'r': {
                std::istringstream iss(optarg);
                double radius;
                while (iss >> radius) {
                    options.radii.push_back(radius);
                }
                break;
            }
            case 'p':
                options.parallel_flag = true;
                break;
            default:
                std::cerr << "Unknown option: " << static_cast<char>(c) << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    // Validate required options
    if (options.input_infiles.empty()) {
        std::cerr << "No input file given (use the -i option)" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (options.outfile.empty()) {
        std::cerr << "No output file given (use the -o option)" << std::endl;
        exit(EXIT_FAILURE);
    }

    return options;
}

#endif // PROGRAM_OPTIONS_HPP