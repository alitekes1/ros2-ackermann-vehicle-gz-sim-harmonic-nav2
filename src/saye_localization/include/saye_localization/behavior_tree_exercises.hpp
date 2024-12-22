#include <behaviortree_cpp_v3/behavior_tree.h>
#include <iostream>
#include <tree_node.h>

// Blackboard üzerinde bir veri saklamak
class LidarSensor : public BT::SyncActionNode
{
public:
    LidarSensor(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<float>("lidar_range")};
    }

    BT::NodeStatus tick() override
    {
        // Lidar'dan veri alıyoruz (örnek veri)
        float lidar_range = 1.2f;              // Lidar verisi
        setOutput("lidar_range", lidar_range); // BlackBoard'a veriyi koyuyoruz

        return BT::NodeStatus::SUCCESS;
    }
};

// ObstacleDetection node'u, lidar verisini blackboard'dan alacak
class ObstacleDetection : public BT::SyncActionNode
{
public:
    ObstacleDetection(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<float>("lidar_range")};
    }

    BT::NodeStatus tick() override
    {
        float lidar_range = 0.0f;
        if (!getInput("lidar_range", lidar_range))
        {
            return BT::NodeStatus::FAILURE;
        }

        if (lidar_range < 1.0f)
        {
            std::cout << "Obstacle detected!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "No obstacle detected." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// Ağaç ve Blackboard'ı oluşturma
int main()
{
    // Blackboard oluşturuluyor
    auto blackboard = BT::Blackboard::create();
    // BT ağacını oluşturuyoruz
    BT::Tree tree;
    tree.setBlackboard(blackboard);

    // Node'ları ağaca ekliyoruz
    BehaviorTreeFactory factory;
    factory.registerNodeType<LidarSensor>("LidarSensor");
    factory.registerNodeType<ObstacleDetection>("ObstacleDetection");

    // Bir davranış ağacı oluşturuyoruz
    BT::TreeNode *root_node = factory.createTreeFromFile("tree.xml");

    // Ağacı çalıştırıyoruz
    tree.tickRoot();
}
