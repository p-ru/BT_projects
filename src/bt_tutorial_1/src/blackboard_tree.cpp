#include "bt_tutorial_1/custom_nodes.h"
#include <ros/package.h>


int main() { 
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");
    //std::string const package_path = ros::package::getPath("bt_tutorial_1");

    auto tree = factory.createTreeFromFile("/home/droneproject/BT_projects/src/bt_tutorial_1/config/my_tree.xml");
    BT::StdCoutLogger logger_cout(tree);
    tree.tickRootWhileRunning();

    return 0;
}
