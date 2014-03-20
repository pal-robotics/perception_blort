/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2012, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @file trackerinterface.h
 * @author Bence Magyar
 * @date March 2012
 * @version 0.1
 * @brief This class describes an interface derived from the blort tracker as a complete application.
 * It defines the system logic of recovery and track modes and confidence states.
 * @namespace pal_blort
 */

#ifndef BLORTTRACKER_H
#define BLORTTRACKER_H

#include <opencv2/core/core.hpp>
#include <ros/console.h> //TODO: remove

namespace blort_ros
{
    enum tracker_mode
    {
        TRACKER_RECOVERY_MODE,
        TRACKER_TRACKING_MODE,
        TRACKER_LOCKED_MODE
    };

    enum tracker_confidence
    {
        TRACKER_CONF_GOOD,
        TRACKER_CONF_FAIR,
        TRACKER_CONF_LOST
    };

    class TrackerInterface
    {
    protected:
        std::vector<tracker_mode> current_modes;
        std::vector<tracker_confidence> current_confs;
        cv::Mat last_image;

    public:
        TrackerInterface()
        {
        }
        virtual void track() = 0;
        virtual void recovery() = 0;
        void process(cv::Mat img)
        {
            last_image = img;
            tracker_mode current_mode = TRACKER_RECOVERY_MODE;
            for(size_t i = 0; current_modes.size(); ++i)
            {
                if(current_modes[i] > current_mode)
                {
                    //FIXME tracking and locked mode are equivalent for now
                    current_mode = current_modes[i];
                    break;
                }
            }
            switch(current_mode)
            {
            case TRACKER_RECOVERY_MODE:
                recovery();
                break;
            case TRACKER_TRACKING_MODE:
                track();
                break;
            case TRACKER_LOCKED_MODE:
                track(); //FIXME a bit hacky
                break;
            }
        }
        virtual void switchToTracking(size_t id){ current_confs[id] = TRACKER_CONF_FAIR; current_modes[id] = TRACKER_TRACKING_MODE; }
        virtual void switchToRecovery(size_t id){ current_confs[id] = TRACKER_CONF_LOST; current_modes[id] = TRACKER_RECOVERY_MODE; }
        virtual void reset(size_t id){ switchToRecovery(id); }
        const std::vector<tracker_mode> & getModes(){ return current_modes; }
        const std::vector<tracker_confidence> & getConfidence(){ return current_confs; }
    };
}

#endif // BLORTTRACKER_H
