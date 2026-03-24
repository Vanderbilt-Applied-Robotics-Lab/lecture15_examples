#include <arun_reg/arun_reg.h>

ArunReg::ArunReg() : Node("arun_reg")
{
    // setup robot points
    
    // setup workpiece points
}

void ArunReg::registerPoints()
{
    // compute Centroids

    // offset from centroids

    // create H

    // Compute SVD
    // compute R

    // check determinate
        // compute frame offset
        // print TRE and FRE
}

Eigen::Vector3d ArunReg::computeCentroid(std::array<Eigen::Vector3d,3> & points)
{

}

void ArunReg::createH()
{

}

double ArunReg::computeFRE()
{

}

double ArunReg::computeTRE()
{
    
}

int main(int argc, char * argv[])
{
    // initialize the node
    rclcpp::init(argc, argv);

    // create instance of class
    auto node = std::make_shared<ArunReg>();

    // MAIN LOOP
    node->registerPoints();
    rclcpp::spin_some(node); // updates publishers and subscribers
    rclcpp::shutdown();
    return 0;
}