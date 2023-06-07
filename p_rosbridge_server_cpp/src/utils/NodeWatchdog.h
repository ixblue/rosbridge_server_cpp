#pragma once

#include <atomic>
#include <mutex>
#include <ros/ros.h>
#include <thread>

/*!
 * \brief The NodeWatchdog class will std::abort the process if the ROS event loop is
 * frozen.
 * Check README.md for usage details
 */

class NodeWatchdog
{
public:
    NodeWatchdog(double timeout_secs, ros::NodeHandle nh = ros::NodeHandle("~"));
    ~NodeWatchdog();

    /*!
     * \brief tick is used to indicate the process is still alive
     * You can call this method during slow tasks
     */
    void tick();

private:
    void onTickTimerCB(const ros::SteadyTimerEvent& ev);

    ros::NodeHandle m_nh; // this one is important, because ROS will shutdown if all
                          // nodehandles are deleted
    ros::SteadyTimer m_timer;

    std::thread m_watchdogThread;

    std::mutex m_last_alive_stamp_mutex;
    ros::SteadyTime m_last_alive_stamp;

    std::atomic<bool> m_shutdown{false};
};
