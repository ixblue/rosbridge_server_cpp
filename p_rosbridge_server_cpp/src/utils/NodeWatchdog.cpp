#include <fstream>
#include "NodeWatchdog.h"

// Detect if the application is running inside a debugger.
// source : https://stackoverflow.com/a/69842462
static bool beingTraced()
{
    std::ifstream sf("/proc/self/status");
    std::string s;
    while(sf >> s)
    {
        if(s == "TracerPid:")
        {
            int pid;
            sf >> pid;
            return pid != 0;
        }
        std::getline(sf, s);
    }

    return false;
}

NodeWatchdog::NodeWatchdog(double timeout_secs, ros::NodeHandle nh) : m_nh(nh)
{
    bool watchdogEnabled = nh.param<bool>("watchdog_enabled", true);
    double watchdogTimeout_s = nh.param<double>("watchdog_timeout", timeout_secs);
    double watchdogInitialTimeout_s =
        nh.param<double>("watchdog_initial_timeout", watchdogTimeout_s);

    if(watchdogInitialTimeout_s < watchdogTimeout_s)
    {
        watchdogInitialTimeout_s = watchdogTimeout_s;
    }

    if(watchdogEnabled)
    {
        m_last_alive_stamp = ros::SteadyTime::now();
        m_timer = nh.createSteadyTimer(ros::WallDuration(watchdogTimeout_s / 5.),
                                       &NodeWatchdog::onTickTimerCB, this, false, true);

        m_watchdogThread = std::thread([this, watchdogTimeout_s,
                                        watchdogInitialTimeout_s] {
            auto sleepDuration = std::chrono::milliseconds(
                static_cast<int64_t>((watchdogTimeout_s * 1000) / 5.));

            ros::SteadyTime startupStamp = ros::SteadyTime::now();

            ros::WallDuration timeoutDuration = ros::WallDuration(watchdogTimeout_s);
            ros::WallDuration initialTimeoutDuration =
                ros::WallDuration(watchdogInitialTimeout_s);

            while(!ros::isShuttingDown() && !m_shutdown)
            {
                std::this_thread::sleep_for(sleepDuration);
                // check timestamp
                ros::WallDuration durSinceLastTick;
                {
                    std::lock_guard<std::mutex> lock(m_last_alive_stamp_mutex);
                    durSinceLastTick = ros::SteadyTime::now() - m_last_alive_stamp;
                }

                if(durSinceLastTick > timeoutDuration)
                {
                    ros::WallDuration durSinceStartup =
                        ros::SteadyTime::now() - startupStamp;
                    if(durSinceStartup > initialTimeoutDuration)
                    {
                        if(!beingTraced())
                        {
                            ROS_ERROR("watchdog detected application freeze, stopping "
                                      "node ...");

#if 0
                            // Give ROS some time to inform the rosmaster it will be
                            // killed
                            // But this can also make debug of ROS freezes more difficult
                            ros::requestShutdown(); // this is thread-safe, it will just
                                                    // set a global "static bool" to true
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                            ROS_ERROR(
                                "watchdog detected application freeze, node will be "
                                "killed NOW");
#endif

                            // abort will generate a coredump if the host is correctly
                            // configured
                            std::abort();
                        }
                        else
                        {
                            ROS_WARN_ONCE(
                                "watchdog detected application freeze, but an active "
                                "debugger is detected: no action is taken");
                        }
                    }
                }
            }
        });
    }
}

NodeWatchdog::~NodeWatchdog()
{
    m_shutdown = true;
    if(m_watchdogThread.joinable())
    {
        m_watchdogThread.join();
    }
}

void NodeWatchdog::tick()
{
    std::lock_guard<std::mutex> lock(m_last_alive_stamp_mutex);
    m_last_alive_stamp = ros::SteadyTime::now();
}

void NodeWatchdog::onTickTimerCB(const ros::SteadyTimerEvent& ev)
{
    (void)ev;
    tick();
}
