#pragma once 
#include <iostream>
#include <vector>
#include <utility>
#include <Eigen/Core>

// X,Y,Z octant1: Top Back Right
// -X,Y,Z octant2: Top Back Left
// -X,-Y,Z octant3: Top Front Left
// X,-Y,Z octant4: Top Front Right
// X,Y,-Z octant5: Bottom Back Right
// -X,Y,-Z octant6: Bottom Back Left
// -X,-Y,-Z octant7: Bottom Front Left
// X,-Y,-Z octant8: Bottom Front Right

struct octnode{

    Eigen::MatrixXd bounding_box;
    octnode* children[8]; 
    octnode* parent;

    int depth;
    int max_depth;
    bool leaf;
    bool obstacle;

    octnode();
    octnode(Eigen::MatrixXd bounding_box, octnode* parent ,int max_depth, int depth, bool obstacle);

    int num_children();

};

// Reminder, when doing odd shapes, if the child class is one -1 dimensions (aka, it cannot be split well so compromises have to be made)
// invalid nodes should be considered obstacles, that way, if the other nodes which are legitimate and have obstacles, the parent node will
// also be considered to have an obstacle.

class octtree{

    private:

        octnode* head;
        
        // Resolutions of the smallest subdivision of the area
        int subdivisions;


    public:

        // Constructions 
        octtree();
        // space bounds is a 3x2 matrix, column 0 represents the low, column 1 represents the higher bound.
        octtree(Eigen::MatrixXd space_bounds, int subdivisions);

        // Destructor 
        ~octtree();

        // Add and delete points of collisioins
        void add(Eigen::Vector3d input);
        void del(Eigen::Vector3d input);

        // checks if a sigle point is colliding 
        bool is_collision(Eigen::Vector3d x);

        // Returns area in the defined space which overlaps with the given area matrix
        std::vector<Eigen::MatrixXd> return_overlap_area(Eigen::MatrixXd bounds);

};