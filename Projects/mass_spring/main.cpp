#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <cstdlib>
#include <random>
#include <chrono>
#include <unordered_set>

#include "SimulationDriver.h"


struct pair_hash
{
    template <class T1, class T2>
	std::size_t operator () (std::pair<T1, T2> const &pair) const
	{
        std::size_t h1 = std::hash<T1>()(pair.first);
		std::size_t h2 = std::hash<T2>()(pair.second);

		return h1 ^ h2;
	}
};

int main(int argc, char* argv[])
{
    using T = float;
    constexpr int dim = 3;
    using TV = Eigen::Matrix<T,dim,1>;

    SimulationDriver<T,dim> driver;

    // set up mass spring system
    T youngs_modulus = 5.3;
    T damping_coeff = 2.5; 
    T dt = 0;

    // node data
    std::vector<T> m;
    std::vector<TV> x;
    std::vector<TV> v;
    std::vector<bool> node_is_fixed;

    // segment data
    std::vector<Eigen::Matrix<int,2,1>> segments;
    std::vector<T> rest_length;

    if (argc < 2) 
    {
        std::cout << "Please indicate test case number: 0 (cloth) or 1 (volumetric bunny)" << std::endl;
        exit(0);
    }

    if (strcmp(argv[1], "0") == 0) // cloth case
    {
        // TODO
        /* 
            1. Create node data: position, mass, velocity
            2. Fill segments and rest_length, including struct springs, shearing springs and bending springs.
            3. Choose proper youngs_modulus, damping_coeff and dt.
            4. Set boundary condition (node_is_fixed) and helper function (to achieve moving boundary condition).
            5. Generate quad mesh for rendering.
        */

        
        std::cout << "adding points 1" << std::endl;
        for (int i = 0; i < 64; i++) {
            for (int j = 0; j < 64; j++) {
                x.push_back(TV(float(j), 0.0f, float(63-i)));
            }
        }
        
        std::cout << "got here now" << std::endl;

        node_is_fixed.push_back(true);
        for (int i = 1; i < 4095; i++) {
            node_is_fixed.push_back(false);
        }
        node_is_fixed.push_back(true);

        for (unsigned long int i = 0; i < x.size(); i++) {
            m.push_back(18.0f);
            v.push_back(TV(0.0f, 0.0f, 0.f));
        } 

        std::cout << "got here now 2" << std::endl;

    
        for (int i = 0; i < 64; i++) {
            for (int k = 0; k < 63; k++) {
                segments.push_back(Eigen::Matrix<int,2,1>(64*i + k, 64*i + k + 1));
            }
        }

        std::cout << "got here now 3" << std::endl;


        for (int i = 0; i < 64; i++) {
            for (int k = 0; k < 63; k++) {
                segments.push_back(Eigen::Matrix<int,2,1>(64*k + i, 64*(k + 1) + i));
            }
        }

        std::cout << "got here now 4" << std::endl;


        for (int i = 0; i < 64; i++) {
            for (int k = 0; k < 60; k++) {
                segments.push_back(Eigen::Matrix<int,2,1>(64*i + k, 64*i + k + 2));
                segments.push_back(Eigen::Matrix<int,2,1>(64*i + k + 1, 64*i + k + 3));
            }
        }

        for (int i = 0; i < 64; i++) {
            for (int k = 0; k < 60; k++) {
                segments.push_back(Eigen::Matrix<int,2,1>(64*k + i, 64*(k + 2) + i));
                segments.push_back(Eigen::Matrix<int,2,1>(64*(k + 1) + i, 64*(k + 3) + i));
            }
        }

        for (int i = 0; i < 63; i++) {
            for (int k = 0; k < 63; k++) {
                segments.push_back(Eigen::Matrix<int,2,1>(64*i + k, 64*i + k + 65));
                segments.push_back(Eigen::Matrix<int,2,1>(64*i + k + 1, 64*i + k + 63));
            }
        }

        std::cout << "got here now 5" << std::endl;



 

        for (unsigned long int i = 0; i < segments.size(); i++) {
            //std::cout << "started loop " << std::endl;
            Eigen::Matrix<int, 2, 1> restd = segments[i];
            //std::cout << "ya boi made it" << std::endl;
            T restdn = restd.norm();
            //std::cout << "restdn: " << restdn << std::endl;
            rest_length.push_back(restdn);
        }

        std::cout << "rest lengths added" << std::endl;

        std::ofstream myfile;
        myfile.open ("cloth.obj");
        for (unsigned long int i = 0; i < x.size(); i++) {
            myfile << "v"  << " " << x[i][0] << " " << x[i][1] << " " << x[i][2] << "\n";
        }

        std::cout << "points added to obj" << std::endl;

        for (unsigned long int i = 1; i <= x.size() - 65; i++) {
            myfile << "f"  << " " << i << " " << i+64 << " " << i+65 << " " << i+1 << "\n";
        }
        myfile.close();

        driver.helper = [&](T t, T dt) {
            // TODO
            for (unsigned long int i = 0; i < x.size(); i++) {
                if (node_is_fixed[i]) {
                    driver.ms.v[i] += TV(cosf(t), 0.0f, 0.0f);
                    driver.ms.x[i] += v[i] * dt;
                }
            }
        };
        driver.test="cloth";
    }

    else if (strcmp(argv[1], "1") == 0) // volumetric bunny case
    { 
        // TODO
        /* 
            1. Create node data from data/points: The first line indicates the number of points and dimension (which is 3). 
            2. Fill segments and rest_length from data/cells: The first line indicates the number of tetrahedra and the number of vertices of each tet (which is 6). Each edge in this tetrahedral mesh will be a segment. Be careful not to create duplicate edges. 
            3. Choose proper youngs_modulus, damping_coeff, dt; 
            4. Set boundary condition (node_is_fixed) and helper function (to achieve moving boundary condition).
        */

        std::cout << "shrampies start" << std::endl;

        std::ifstream inFile("data/points.txt");
        float x1, x2, x3;
        std::string fline;
        std::getline(inFile, fline);
        std::cout << "shrampies loop" << std::endl;
        while ( inFile >> x1 >> x2 >> x3) {
            
            TV new_particle = TV(x1, x2, x3);
            x.push_back(new_particle);
            std::cout << "shrampies done" << std::endl;
        }

        node_is_fixed.push_back(true);
        for (int i = 1; i < 4095; i++) {
            node_is_fixed.push_back(false);
        }
        node_is_fixed.push_back(true);

        for (unsigned long int i = 0; i < x.size(); i++) {
            m.push_back(1.0f);
            v.push_back(TV(0.0f, 0.0f, 0.f));
        } 

        

        std::fstream inFile2("data/cells");
        int e1, e2, e3, e4;
        std::string line1;
        std::getline(inFile2, line1);
        std::unordered_set<std::pair<int,int>, pair_hash> temp_seg;
        //inFile2 >> e1 >> e2 >> e3 >> e4;
        //std::cout << "e1: " << e1 << std::endl;
        std::cout << "before loop" << std::endl;
        while (inFile2 >> e1 >> e2 >> e3 >> e4) {
            std::cout << "loop begun" << std::endl;
            std::pair<int,int> seg1 = std::make_pair(e1, e2);
            std::pair<int,int> seg2 = std::make_pair(e2, e3);
            std::pair<int,int> seg3 = std::make_pair(e3, e4);
            std::pair<int,int> seg4 = std::make_pair(e4, e1);
            
            temp_seg.insert(seg1);
            temp_seg.insert(seg2);
            temp_seg.insert(seg3);
            temp_seg.insert(seg4);
            std::cout << "seg1: " << "(" << seg1.first << ", " << seg1.second << std::endl;
            std::cout << "segs inserted into map" << std::endl;

        }

        for (auto itr = temp_seg.begin(); itr != temp_seg.end(); ++itr) {
            Eigen::Matrix<int,2,1> seg = Eigen::Matrix<int,2,1>((*itr).first, (*itr).second);
            segments.push_back(seg);
            std::cout << "seg: " << "(" << seg[0] << ", " << seg[1] << ")" << std::endl;
        }

        for (unsigned long int i = 0; i < segments.size(); i++) {
            T restd = (segments[i]).norm();
            rest_length.push_back(restd);
        }




        driver.helper = [&](T t, T dt) {
            // TODO
            for (unsigned long int i = 0; i < x.size(); i++) {
                if (node_is_fixed[i]) {
                    driver.ms.v[i] += TV(cosf(t), 0.0f, 0.0f);
                    driver.ms.x[i] += v[i] * dt;
                }
            }
        };
        driver.test="bunny";
    }

    else {
        std::cout << "Wrong case number!" << std::endl;
        exit(0);
    }

    // simulate
    
    driver.dt = dt;
    driver.ms.segments = segments;
    driver.ms.m = m;
    driver.ms.v = v;
    driver.ms.x = x;
    driver.ms.youngs_modulus = youngs_modulus;
    driver.ms.damping_coeff = damping_coeff;
    driver.ms.node_is_fixed = node_is_fixed;
    driver.ms.rest_length = rest_length;

    driver.run(120);

    return 0;
}
