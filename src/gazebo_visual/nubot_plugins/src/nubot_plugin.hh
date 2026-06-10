#ifndef NUBOT_PLUGIN_HH
#define NUBOT_PLUGIN_HH

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Types.hh>
#include <gz/transport/Node.hh>


enum nubot_state
{
    CHASE_BALL,
    DRIBBLE_BALL,
    KICK_BALL,
    RESET
}

enum nubot_substate
{
    MOVE_BALL,
    ROTATE_BALL
};

namespace nubot_plugin
{
    class nubot_plugin final
    {
    private:
        /* data */
    public:
        nubot_plugin() = default;
        ~nubot_plugin() = default ;

        // ISystemConfigure: 初始化配置
        void Configure(
            const gz::sim::Entity &_entity,
            const std::shared_ptr<const sdf::Element> &_sdf,
            gz::sim::EntityComponentManager &_ecm,
            gz::sim::EventManager &_eventMgr) override;

    
}


#endif