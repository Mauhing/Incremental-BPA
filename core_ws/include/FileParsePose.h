/**
 * @file FileParsePose.h
 * @copyright This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * @author Mau Hing Yip mauhingyip@hotmail.com
 * @date 2025-06-12
 */


#ifndef FILE_PARSE_POSE_H
#define FILE_PARSE_POSE_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <Eigen/Dense>

struct Point3D {
    double x, y, z;
    Eigen::Vector3d normal;
};

struct SensorFrame {
    Eigen::Matrix<double, 3, 4> pose;
    std::vector<Point3D> points;

    void calculateNormals() {
        Eigen::Vector3d sensor_pos = pose.block<3,1>(0,3);
        
        for (auto& point : points) {
            Eigen::Vector3d point_pos(point.x, point.y, point.z);
            point.normal = (sensor_pos - point_pos).normalized();
        }
    }

    void defaultPose() {
        pose.setZero();
        pose(0, 0) = 1.0; 
        pose(1, 1) = 1.0; 
        pose(2, 2) = 1.0;
    }
};

class OfflineDataParser {
public:
    static std::vector<SensorFrame> parseFile(const std::string& filename, bool with_normal = false) {
        std::vector<SensorFrame> frames;
        std::ifstream file(filename);
        
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file: " + filename);
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line == "---") {
                if (with_normal) {
                    SensorFrame frame = parseSensorFrameWithNormal(file);
                    frames.push_back(frame);
                } else {
                    SensorFrame frame = parseSensorFrame(file);
                    frame.calculateNormals();
                    frames.push_back(frame);
                }
            }
        }

        return frames;
    }

private:
    static SensorFrame parseSensorFrame(std::ifstream& file) {
        SensorFrame frame;
        std::string line;
        
        std::getline(file, line);
        if (line != "pose:") {
            throw std::runtime_error("Expected 'pose:', got: " + line);
        }

        for (int i = 0; i < 3; ++i) {
            std::getline(file, line);
            std::istringstream iss(line);
            for (int j = 0; j < 4; ++j) {
                iss >> frame.pose(i, j);
            }
        }

        std::getline(file, line);
        if (line != "points:") {
            throw std::runtime_error("Expected 'points:', got: " + line);
        }

        while (std::getline(file, line) && line != "---") {
            std::istringstream iss(line);
            Point3D point;
            iss >> point.x >> point.y >> point.z;
            frame.points.push_back(point);
        }

        return frame;
    }

    static SensorFrame parseSensorFrameWithNormal(std::ifstream& file) {
        SensorFrame frame;
        std::string line;
        
        frame.defaultPose();

        std::getline(file, line);
        if (line != "points:") {
            throw std::runtime_error("Expected 'points:', got: " + line);
        }

        while (std::getline(file, line) && line != "---") {
            std::istringstream iss(line);
            Point3D point;
            iss >> point.x >> point.y >> point.z >> point.normal.x() >> point.normal.y() >> point.normal.z();
            frame.points.push_back(point);
        }

        return frame;
    }
};

#endif // FILE_PARSE_POSE_H